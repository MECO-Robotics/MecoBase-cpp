#include "base/Replay.h"
#include "base/FrameRecorder.h"
#include "base/AdvantageKitLog.h"
#include "base/AdvantageKitLogReader.h"
#include "base/AdvantageKitReplayAdapter.h"
#include <atomic>
#include <algorithm>
#include <array>
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/RobotController.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>
#include <networktables/NetworkTableInstance.h>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <map>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>
#include <utility>
#include <vector>
#ifndef __FRC_ROBORIO__
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#endif
#if __has_include("BuildProvenance.h")
#include "BuildProvenance.h"
#else
#define FRC_BUILD_REVISION "unknown"
#define FRC_BUILD_DIRTY "unknown"
#define FRC_BUILD_SOURCE_HASH "unknown"
#endif
namespace base::replay
{
namespace
{
using Json = wpi::json;
bool initialized = false, active = false, framed = false;
bool pathPlannerAssetsVerified = false;
std::exception_ptr initializationError;
uint64_t index = 0;
size_t cursor = 0;
std::atomic<size_t> pendingReplayBytes{0};
Json frame;
Json provenance;
bool metadataWritten = false;
std::filesystem::path outputPath;
std::vector<Json> source;
std::shared_ptr<wpi::log::StringLogEntry> frames, metadata, recordingFault, recordingStatus, inputSnapshot,
    outputSnapshot;
std::unique_ptr<FrameRecorder> recorder;
FrameRecorder::Stats finalRecordingStats;
std::map<std::string, std::pair<int, std::string>> typedEntries;
[[noreturn]] void Fail(const std::string &message)
{
    throw std::runtime_error("Replay: " + message);
}
std::optional<std::string> ReadAssetFile(const std::string &path, std::size_t maxBytes)
{
    const std::filesystem::path assetPath{path};
    std::error_code error;
    const bool exists = std::filesystem::exists(assetPath, error);
    if (error)
        throw std::runtime_error("unable to inspect replay asset: " + path + ": " + error.message());
    if (!exists)
        return std::nullopt;
    if (std::filesystem::is_symlink(assetPath, error) || error)
        throw std::runtime_error("replay asset cannot be a symlink: " + path);
    const auto size = std::filesystem::file_size(assetPath, error);
    if (error || !std::filesystem::is_regular_file(assetPath, error) || error)
        throw std::runtime_error("unable to read replay asset: " + path);
    if (size > maxBytes || size > static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max()))
        throw std::runtime_error("replay asset exceeds size limit: " + path);
    std::ifstream stream{assetPath, std::ios::binary};
    if (!stream)
        throw std::runtime_error("unable to open replay asset: " + path);
    std::string contents(static_cast<std::size_t>(size), '\0');
    if (size > 0) {
        stream.read(contents.data(), static_cast<std::streamsize>(size));
        if (stream.gcount() != static_cast<std::streamsize>(size))
            throw std::runtime_error("unable to read complete replay asset: " + path);
    }
    return contents;
}
Json ReadAssetDirectory(const std::string &path, std::size_t maxTotalBytes)
{
    const std::filesystem::path root{path};
    std::error_code error;
    if (!std::filesystem::exists(root, error)) {
        if (error)
            throw std::runtime_error("unable to inspect replay asset directory: " + path + ": " + error.message());
        return Json{{"present", false}};
    }
    if (std::filesystem::is_symlink(root, error) || error)
        throw std::runtime_error("replay asset directory cannot be a symlink: " + path);
    if (!std::filesystem::is_directory(root, error) || error)
        throw std::runtime_error("replay asset path is not a directory: " + path);
    std::vector<std::filesystem::path> files;
    std::filesystem::recursive_directory_iterator it{root, error}, end;
    if (error)
        throw std::runtime_error("unable to enumerate replay asset directory: " + path + ": " + error.message());
    while (it != end) {
        if (error)
            throw std::runtime_error("unable to enumerate replay asset directory: " + path + ": " + error.message());
        if (it->is_symlink(error))
            throw std::runtime_error("symlinks are not allowed in replay asset directories: " + it->path().string());
        if (error)
            throw std::runtime_error("unable to inspect replay asset: " + it->path().string());
        if (it->is_regular_file(error)) {
            if (error)
                throw std::runtime_error("unable to inspect replay asset: " + it->path().string());
            files.push_back(it->path());
        } else if (error) {
            throw std::runtime_error("unable to inspect replay asset: " + it->path().string());
        }
        it.increment(error);
        if (error)
            throw std::runtime_error("unable to enumerate replay asset directory: " + path + ": " + error.message());
    }
    std::sort(files.begin(), files.end());
    Json contents = Json::object();
    std::size_t totalBytes = 0;
    for (const auto &file : files) {
        const auto relative = std::filesystem::relative(file, root, error);
        if (error)
            throw std::runtime_error("unable to resolve replay asset path: " + file.string());
        const auto bytes = ReadAssetFile(file.string(), maxTotalBytes);
        if (!bytes || bytes->size() > maxTotalBytes - totalBytes)
            throw std::runtime_error("replay asset directory exceeds size limit: " + path);
        totalBytes += bytes->size();
        contents[relative.generic_string()] = *bytes;
    }
    return Json{{"present", true}, {"files", std::move(contents)}};
}
Json ReadAssetDirectoryManifest(const std::string &path, std::size_t maxTotalBytes)
{
    const std::filesystem::path root{path};
    std::error_code error;
    if (!std::filesystem::exists(root, error)) {
        if (error)
            throw std::runtime_error("unable to inspect replay asset directory: " + path + ": " + error.message());
        return Json{{"present", false}};
    }
    if (std::filesystem::is_symlink(root, error) || error)
        throw std::runtime_error("replay asset directory cannot be a symlink: " + path);
    if (!std::filesystem::is_directory(root, error) || error)
        throw std::runtime_error("replay asset path is not a directory: " + path);
    std::vector<std::filesystem::path> files;
    std::filesystem::recursive_directory_iterator it{root, error}, end;
    if (error)
        throw std::runtime_error("unable to enumerate replay asset directory: " + path + ": " + error.message());
    while (it != end) {
        if (error)
            throw std::runtime_error("unable to enumerate replay asset directory: " + path + ": " + error.message());
        if (it->is_symlink(error))
            throw std::runtime_error("symlinks are not allowed in replay asset directories: " + it->path().string());
        if (error)
            throw std::runtime_error("unable to inspect replay asset: " + it->path().string());
        if (it->is_regular_file(error)) {
            if (error)
                throw std::runtime_error("unable to inspect replay asset: " + it->path().string());
            files.push_back(it->path());
        } else if (error) {
            throw std::runtime_error("unable to inspect replay asset: " + it->path().string());
        }
        it.increment(error);
        if (error)
            throw std::runtime_error("unable to enumerate replay asset directory: " + path + ": " + error.message());
    }
    std::sort(files.begin(), files.end());
    Json manifest = Json::object();
    std::size_t totalBytes = 0;
    for (const auto &file : files) {
        const auto relative = std::filesystem::relative(file, root, error);
        if (error)
            throw std::runtime_error("unable to resolve replay asset path: " + file.string());
        std::ifstream stream{file, std::ios::binary};
        if (!stream)
            throw std::runtime_error("unable to open replay asset: " + file.string());
        std::array<char, 8192> buffer{};
        std::uint64_t hash = 14695981039346656037ull;
        std::size_t fileBytes = 0;
        while (stream) {
            stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const auto count = stream.gcount();
            if (count < 0 || static_cast<std::size_t>(count) > maxTotalBytes - totalBytes ||
                static_cast<std::size_t>(count) > maxTotalBytes - totalBytes - fileBytes)
                throw std::runtime_error("replay asset directory exceeds size limit: " + path);
            fileBytes += static_cast<std::size_t>(count);
            for (std::streamsize index = 0; index < count; ++index) {
                hash ^= static_cast<unsigned char>(buffer[static_cast<std::size_t>(index)]);
                hash *= 1099511628211ull;
            }
        }
        if (!stream.eof())
            throw std::runtime_error("unable to read replay asset: " + file.string());
        totalBytes += fileBytes;
        manifest[relative.generic_string()] = Json{{"size", fileBytes}, {"fnv64", hash}};
    }
    return Json{{"present", true}, {"files", std::move(manifest)}};
}
bool InvalidRecordingStatus(const Json &status, size_t frameCount)
{
    if (status.is_null())
        return false;
    if (status.value("rejected", 0) != 0 || status.value("failed", 0) != 0)
        return true;
    const auto written = status.value("written", static_cast<int64_t>(frameCount));
    if (written < 0)
        return true;
    return status.value("final", false) && static_cast<uint64_t>(written) != frameCount;
}
std::string EscapePathSegment(std::string_view value)
{
    std::string escaped;
    escaped.reserve(value.size());
    for (char c : value)
    {
        if (c == '~')
            escaped += "~0";
        else if (c == '/')
            escaped += "~1";
        else
            escaped += c;
    }
    return escaped;
}
int TypedEntry(const std::string &name, std::string_view type)
{
    auto found = typedEntries.find(name);
    if (found != typedEntries.end())
    {
        if (found->second.second != type)
            Fail("replay field changed WPILOG type: " + name);
        return found->second.first;
    }
    const int id = base::logging::GetLog().Start(name, type, base::logging::kAdvantageKitMetadata);
    if (id == 0)
        Fail("failed to start typed WPILOG entry: " + name);
    typedEntries.emplace(name, std::make_pair(id, std::string(type)));
    return id;
}
void AppendTypedJson(std::string_view path, const Json &value, int64_t timestamp)
{
    auto &log = base::logging::GetLog();
    if (value.is_object())
    {
        for (auto it = value.begin(); it != value.end(); ++it)
        {
            const auto child = std::string(path) + "/" + EscapePathSegment(it.key());
            AppendTypedJson(child, it.value(), timestamp);
        }
        return;
    }
    if (value.is_array())
    {
        const auto &array = value;
        const auto lengthPath = std::string(path) + "/length";
        if (array.empty())
        {
            const int id = TypedEntry(lengthPath, "int64");
            log.AppendInteger(id, 0, timestamp);
            return;
        }
        const bool booleans =
            std::all_of(array.begin(), array.end(), [](const Json &item) { return item.is_boolean(); });
        const bool strings = std::all_of(array.begin(), array.end(), [](const Json &item) { return item.is_string(); });
        const bool integers =
            std::all_of(array.begin(), array.end(), [](const Json &item) { return item.is_number_integer(); });
        const bool numbers = std::all_of(array.begin(), array.end(), [](const Json &item) { return item.is_number(); });
        if (booleans)
        {
            std::vector<int> values;
            values.reserve(array.size());
            for (const auto &item : array)
                values.push_back(item.get<bool>() ? 1 : 0);
            const int id = TypedEntry(std::string(path), "boolean[]");
            log.AppendBooleanArray(id, values, timestamp);
            return;
        }
        if (strings)
        {
            std::vector<std::string> values;
            values.reserve(array.size());
            for (const auto &item : array)
                values.push_back(item.get<std::string>());
            const int id = TypedEntry(std::string(path), "string[]");
            log.AppendStringArray(id, values, timestamp);
            return;
        }
        if (integers)
        {
            bool fitSigned = true;
            std::vector<int64_t> values;
            values.reserve(array.size());
            for (const auto &item : array)
            {
                if (item.is_number_unsigned() &&
                    item.get<uint64_t>() > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
                {
                    fitSigned = false;
                    break;
                }
                values.push_back(item.get<int64_t>());
            }
            if (fitSigned)
            {
                const int id = TypedEntry(std::string(path), "int64[]");
                log.AppendIntegerArray(id, values, timestamp);
                return;
            }
        }
        if (numbers)
        {
            std::vector<double> values;
            values.reserve(array.size());
            for (const auto &item : array)
                values.push_back(item.get<double>());
            const int id = TypedEntry(std::string(path), "double[]");
            log.AppendDoubleArray(id, values, timestamp);
            return;
        }
        const int lengthId = TypedEntry(lengthPath, "int64");
        log.AppendInteger(lengthId, static_cast<int64_t>(array.size()), timestamp);
        for (size_t i = 0; i < array.size(); ++i)
        {
            AppendTypedJson(std::string(path) + "/" + std::to_string(i), array[i], timestamp);
        }
        return;
    }
    if (value.is_boolean())
    {
        const int id = TypedEntry(std::string(path), "boolean");
        log.AppendBoolean(id, value.get<bool>(), timestamp);
    }
    else if (value.is_number_integer())
    {
        if (value.is_number_unsigned() &&
            value.get<uint64_t>() > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
        {
            const int id = TypedEntry(std::string(path), "string");
            log.AppendString(id, value.dump(), timestamp);
        }
        else
        {
            const int id = TypedEntry(std::string(path), "int64");
            log.AppendInteger(id, value.get<int64_t>(), timestamp);
        }
    }
    else if (value.is_number())
    {
        const int id = TypedEntry(std::string(path), "double");
        log.AppendDouble(id, value.get<double>(), timestamp);
    }
    else if (value.is_string())
    {
        const int id = TypedEntry(std::string(path), "string");
        log.AppendString(id, value.get<std::string>(), timestamp);
    }
}
void Validate(const Json &j)
{
    if (j.is_number_float() && !std::isfinite(j.get<double>()))
        Fail("nonfinite number");
    if (j.is_structured())
        for (const auto &v : j)
            Validate(v);
}
Json CaptureDS()
{
    Json sticks = Json::array();
    for (int stick = 0; stick < 6; ++stick)
    {
        Json axes = Json::array(), povs = Json::array();
        for (int i = 0; i < frc::DriverStation::GetStickAxisCount(stick); ++i)
            axes.push_back(frc::DriverStation::GetStickAxis(stick, i));
        for (int i = 0; i < frc::DriverStation::GetStickPOVCount(stick); ++i)
            povs.push_back(frc::DriverStation::GetStickPOV(stick, i));
        sticks.push_back({{"axes", axes},
                          {"povs", povs},
                          {"buttons", frc::DriverStation::GetStickButtons(stick)},
                          {"buttonCount", frc::DriverStation::GetStickButtonCount(stick)},
                          {"xbox", frc::DriverStation::GetJoystickIsXbox(stick)},
                          {"type", frc::DriverStation::GetJoystickType(stick)},
                          {"name", frc::DriverStation::GetJoystickName(stick)}});
    }
    auto alliance = frc::DriverStation::GetAlliance();
    return {{"enabled", frc::DriverStation::IsEnabled()},
            {"autonomous", frc::DriverStation::IsAutonomous()},
            {"test", frc::DriverStation::IsTest()},
            {"estop", frc::DriverStation::IsEStopped()},
            {"dsAttached", frc::DriverStation::IsDSAttached()},
            {"fmsAttached", frc::DriverStation::IsFMSAttached()},
            {"alliance", alliance ? (*alliance == frc::DriverStation::Alliance::kRed ? "red" : "blue") : "unknown"},
            {"location", frc::DriverStation::GetLocation().value_or(1)},
            {"joysticks", sticks}};
}
#ifndef __FRC_ROBORIO__
void InstallDS(const Json &ds)
{
    using D = frc::sim::DriverStationSim;
    D::SetEnabled(ds.at("enabled").get<bool>());
    D::SetAutonomous(ds.at("autonomous").get<bool>());
    D::SetTest(ds.at("test").get<bool>());
    D::SetEStop(ds.at("estop").get<bool>());
    D::SetDsAttached(ds.at("dsAttached").get<bool>());
    D::SetFmsAttached(ds.at("fmsAttached").get<bool>());
    int location = ds.at("location").get<int>();
    if (location < 1 || location > 3)
        Fail("invalid alliance location");
    auto alliance = ds.at("alliance").get<std::string>();
    D::SetAllianceStationId(
        alliance == "red"    ? static_cast<HAL_AllianceStationID>(HAL_AllianceStationID_kRed1 + location - 1)
        : alliance == "blue" ? static_cast<HAL_AllianceStationID>(HAL_AllianceStationID_kBlue1 + location - 1)
                             : HAL_AllianceStationID_kUnknown);
    const auto &sticks = ds.at("joysticks");
    if (sticks.size() != 6)
        Fail("expected six joystick ports");
    for (int stick = 0; stick < 6; ++stick)
    {
        const auto &s = sticks.at(stick);
        const auto &axes = s.at("axes"), &povs = s.at("povs");
        if (axes.size() > 12 || povs.size() > 12)
            Fail("joystick dimension out of range");
        D::SetJoystickAxisCount(stick, axes.size());
        D::SetJoystickPOVCount(stick, povs.size());
        for (size_t i = 0; i < axes.size(); ++i)
            D::SetJoystickAxis(stick, i, axes.at(i).get<double>());
        for (size_t i = 0; i < povs.size(); ++i)
            D::SetJoystickPOV(stick, i, povs.at(i).get<int>());
        D::SetJoystickButtons(stick, s.at("buttons").get<uint32_t>());
        D::SetJoystickButtonCount(stick, s.at("buttonCount").get<int>());
        D::SetJoystickIsXbox(stick, s.at("xbox").get<bool>());
        D::SetJoystickType(stick, s.at("type").get<int>());
        D::SetJoystickName(stick, s.at("name").get<std::string>());
    }
    D::NotifyNewData();
    frc::DriverStation::RefreshData();
}
#endif
} // namespace
void InitializeFromEnvironment()
{
    if (initializationError)
        std::rethrow_exception(initializationError);
    if (initialized)
        return;
    initialized = true;
    pathPlannerAssetsVerified = false;
    try
    {
    const char *input = std::getenv("FRC_REPLAY_LOG");
    active = input && *input;
    provenance = {{"schemaVersion", 1},
                  {"buildRevision", FRC_BUILD_REVISION},
                  {"buildDirty", std::string_view(FRC_BUILD_DIRTY) == "true"},
                  {"buildSourceHash", FRC_BUILD_SOURCE_HASH},
                  {"mode", active ? "replay" : "live"}};
    if (active)
    {
        // RobotBase may start the default NT server before our member initialization.
        // Replay uses only process-local tables; remote chooser/tuning writes must
        // never change the recorded-input run or contend with a live simulator.
        auto nt = nt::NetworkTableInstance::GetDefault();
        nt.StopServer();
        nt.StopClient();
        nt.StartLocal();
#ifdef __FRC_ROBORIO__
        Fail("input replay is desktop-only");
#else
        auto buffer = wpi::MemoryBuffer::GetFile(input);
        if (!buffer)
            Fail("cannot read source log");
        uint64_t hash = 14695981039346656037ull;
        for (unsigned char c : (*buffer)->GetBuffer())
        {
            hash ^= c;
            hash *= 1099511628211ull;
        }
        wpi::log::DataLogReader reader(std::move(*buffer));
        if (!reader.IsValid())
            Fail("invalid WPILOG");
        int64_t lastSourceTimestamp = -1;
        auto appendFrame = [&](Json f, uint64_t expectedSequence, int64_t &last) {
            Validate(f);
            if (f.at("schemaVersion") != 1)
                Fail("unsupported frame schema");
            const auto time = f.at("timestampUs").get<int64_t>();
            if (time < 0 || time <= last)
                Fail("frame timestamps must strictly increase");
            last = time;
            if (f.at("sequence").get<uint64_t>() != expectedSequence)
                Fail("frame sequence is not contiguous");
            if (!f.at("inputs").is_object() || !f.at("outputs").is_object() || !f.at("ds").is_object())
                Fail("invalid frame structure");
            source.push_back(std::move(f));
        };
        if (reader.GetExtraHeader() == base::logging::kAdvantageKitHeader)
        {
            const auto akFrames = base::akit::ReadLog(reader);
            bool hasNativeFrames = false;
            Json recordingStatus;
            for (const auto &cycle : akFrames)
            {
                if (const auto *fault = cycle.Find("Replay/RecordingFault"))
                {
                    if (const auto *text = std::get_if<std::string>(&fault->value))
                    {
                        const auto status = Json::parse(*text);
                        if (status.value("rejected", 0) > 0 || status.value("failed", 0) > 0)
                            Fail("source recording reported overflow or serialization failure");
                    }
                }
                if (const auto *status = cycle.Find("Replay/RecordingStatus"))
                    if (const auto *text = std::get_if<std::string>(&status->value))
                        recordingStatus = Json::parse(*text);
                if (const auto *meta = cycle.Find("Replay/Metadata"))
                    if (const auto *text = std::get_if<std::string>(&meta->value))
                        provenance["sourceMetadata"] = Json::parse(*text);
                if (const auto *payload = cycle.Find("Replay/Frames"))
                {
                    const auto *text = std::get_if<std::string>(&payload->value);
                    if (!text)
                        Fail("AdvantageKit Replay/Frames entry is not a string");
                    auto nativeFrame = Json::parse(*text);
                    if (nativeFrame.at("timestampUs").get<int64_t>() != cycle.timestampUs)
                        Fail("AdvantageKit Replay/Frames timestamp does not match its /Timestamp cycle");
                    appendFrame(std::move(nativeFrame), source.size(), lastSourceTimestamp);
                    hasNativeFrames = true;
                }
            }
            if (!hasNativeFrames)
            {
                const char *backendEnv = std::getenv("FRC_DRIVE_BACKEND");
                const std::string backend = backendEnv && *backendEnv ? backendEnv : "phoenix";
                try
                {
                    for (size_t i = 0; i < akFrames.size(); ++i)
                        appendFrame(base::akit::ConvertJavaAdvantageKitFrame(akFrames[i], i, backend), source.size(),
                                    lastSourceTimestamp);
                }
                catch (const std::exception &error)
                {
                    Fail(std::string("cannot map AdvantageKit log to base replay inputs: ") + error.what());
                }
            }
            if (InvalidRecordingStatus(recordingStatus, source.size()))
                Fail("source recording status reports failed or missing frames");
        }
        else
        {
            std::map<int, std::string> entries;
            int64_t last = -1;
            uint64_t sequence = 0;
            Json recordingStatus;
            for (const auto &record : reader)
            {
                if (record.IsStart())
                {
                    wpi::log::StartRecordData start;
                    if (record.GetStartData(&start))
                        entries[start.entry] = std::string(start.name);
                    continue;
                }
                const auto &name = entries[record.GetEntry()];
                std::string_view value;
                if (name == "Replay/RecordingFault" || name == "/Replay/RecordingFault")
                    Fail("source recording reported overflow or serialization failure");
                if ((name == "Replay/RecordingStatus" || name == "/Replay/RecordingStatus") && record.GetString(&value))
                    recordingStatus = Json::parse(value);
                if ((name == "Replay/Metadata" || name == "/Replay/Metadata") && record.GetString(&value))
                    provenance["sourceMetadata"] = Json::parse(value);
                if (name != "/Replay/Frames" && name != "Replay/Frames")
                    continue;
                if (!record.GetString(&value))
                    continue;
                appendFrame(Json::parse(value), sequence++, last);
            }
            if (InvalidRecordingStatus(recordingStatus, source.size()))
                Fail("source recording status reports failed or missing frames");
        }
        if (source.empty())
            Fail("source has no replay input frames");
        provenance["sourceFile"] = std::filesystem::absolute(input).string();
        provenance["sourceFileHashFnv1a64"] = std::to_string(hash);
        if (const char *sha = std::getenv("FRC_REPLAY_SOURCE_SHA256"))
        {
            const std::string digest(sha);
            if (digest.size() != 64 || digest.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos)
                Fail("invalid source SHA256 metadata");
            provenance["sourceFileSha256"] = digest;
        }
        const char *output = std::getenv("FRC_REPLAY_OUTPUT");
        if (!output || !*output)
            Fail("FRC_REPLAY_OUTPUT must name a new WPILOG file");
        auto path = std::filesystem::absolute(output);
        outputPath = path;
        if (std::filesystem::exists(path))
            Fail("output already exists; refusing overwrite");
        std::filesystem::create_directories(path.parent_path());
        base::logging::StartAdvantageKitLog(path.parent_path().string(), path.filename().string(), .01);
        std::cout << "[Replay] Loaded " << source.size() << " input frames; new outputs: " << path << std::endl;
#endif
    }
    else
    {
#ifdef __FRC_ROBORIO__
        base::logging::StartAdvantageKitLog("/home/lvuser/logs");
#else
        base::logging::StartAdvantageKitLog("logs");
#endif
    }
    auto &log = base::logging::GetLog();
    metadata =
        std::make_shared<wpi::log::StringLogEntry>(log, "/Replay/Metadata", base::logging::kAdvantageKitMetadata);
    frames = std::make_shared<wpi::log::StringLogEntry>(log, "/Replay/Frames", base::logging::kAdvantageKitMetadata);
    recordingFault =
        std::make_shared<wpi::log::StringLogEntry>(log, "/Replay/RecordingFault", base::logging::kAdvantageKitMetadata);
    recordingStatus = std::make_shared<wpi::log::StringLogEntry>(log, "/Replay/RecordingStatus",
                                                                 base::logging::kAdvantageKitMetadata);
    inputSnapshot =
        std::make_shared<wpi::log::StringLogEntry>(log, "/Replay/Inputs", base::logging::kAdvantageKitMetadata);
    outputSnapshot =
        std::make_shared<wpi::log::StringLogEntry>(log, "/Replay/Outputs", base::logging::kAdvantageKitMetadata);
    recorder = std::make_unique<FrameRecorder>(
        [entry = frames, metadataEntry = metadata, inputEntry = inputSnapshot, outputEntry = outputSnapshot,
         metadataPayload = provenance.dump()](const Json &record, int64_t timestamp) mutable {
            base::logging::AppendTimestamp(timestamp);
            if (!metadataWritten)
            {
                metadataEntry->Append(metadataPayload, timestamp);
                metadataWritten = true;
            }
            for (auto it = record.at("inputs").begin(); it != record.at("inputs").end(); ++it)
                AppendTypedJson("/ReplayInputs/" + it.key(), it.value(), timestamp);
            for (auto it = record.at("outputs").begin(); it != record.at("outputs").end(); ++it)
                AppendTypedJson("/ReplayOutputs/" + it.key(), it.value(), timestamp);
            inputEntry->Append(record.at("inputs").dump(), timestamp);
            outputEntry->Append(record.at("outputs").dump(), timestamp);
            const auto payload = record.dump();
            entry->Append(payload, timestamp);
            if (active)
                pendingReplayBytes.fetch_add(payload.size(), std::memory_order_relaxed);
        },
        [entry = recordingFault](FrameRecorder::Stats status) {
            entry->Append(Json{{"rejected", status.rejected}, {"failed", status.failed}}.dump());
        });
    }
    catch (...)
    {
        // A partial replay initialization must never permit later IO creation.
        initializationError = std::current_exception();
        throw;
    }
}
bool Active()
{
    if (initializationError)
        std::rethrow_exception(initializationError);
    // Preserve live logging startup, but establish replay before any IO checks
    // Active(), even when the robot's members precede its constructor body.
    if (!initialized)
    {
        const char *input = std::getenv("FRC_REPLAY_LOG");
        if (input && *input)
            InitializeFromEnvironment();
    }
    return active;
}
bool FrameActive()
{
    return framed;
}
uint64_t FrameIndex()
{
    return index;
}
void BeginFrame()
{
    if (framed)
        return;
    if (!initialized)
        InitializeFromEnvironment();
    if (active)
    {
        if (cursor >= source.size())
            Fail("frame beyond EOF");
        frame = std::move(source.at(cursor));
        frame["outputs"] = Json::object();
        index = frame.at("sequence").get<uint64_t>();
    }
    else
        frame = {{"schemaVersion", 1}, {"sequence", index},        {"timestampUs", frc::RobotController::GetFPGATime()},
                 {"ds", CaptureDS()},  {"inputs", Json::object()}, {"outputs", Json::object()}};
    framed = true;
}
Json ProcessInputs(std::string_view key, std::function<Json()> live)
{
    if (!framed)
    {
        if (!active)
            return live();
        BeginFrame();
    }
    auto name = std::string(key);
    if (!frame["inputs"].contains(name))
    {
        if (active)
            Fail("missing required inputs: " + name);
        auto value = live();
        Validate(value);
        frame["inputs"][name] = std::move(value);
    }
    return frame["inputs"].at(name);
}
std::optional<std::string> ProcessFileInput(std::string_view key, std::string_view path,
                                            std::size_t maxBytes)
{
    if (!framed)
        Fail("file input requested outside a frame: " + std::string{key});
    const std::string filePath{path};
    const auto input = ProcessInputs(key, [filePath, maxBytes] {
        const auto contents = ReadAssetFile(filePath, maxBytes);
        if (!contents)
            return Json{{"present", false}};
        return Json{{"present", true}, {"contents", *contents}};
    });
    if (!input.is_object() || !input.contains("present") || !input.at("present").is_boolean())
        throw std::runtime_error("invalid recorded replay asset: " + std::string{key});
    if (!input.at("present").get<bool>())
        return std::nullopt;
    if (!input.contains("contents") || !input.at("contents").is_string())
        throw std::runtime_error("missing recorded replay asset bytes: " + std::string{key});
    auto contents = input.at("contents").get<std::string>();
    if (contents.size() > maxBytes)
        throw std::runtime_error("recorded replay asset exceeds size limit: " + std::string{key});
    return contents;
}
void VerifyFileInput(std::string_view key, std::string_view path, std::size_t maxBytes)
{
    if (!framed)
        Fail("file verification requested outside a frame: " + std::string{key});
    const auto recorded = ProcessFileInput(key, path, maxBytes);
    if (Active() && recorded != ReadAssetFile(std::string{path}, maxBytes))
        Fail("deploy asset differs from recording: " + std::string{path});
}
void VerifyDirectoryInput(std::string_view key, std::string_view path, std::size_t maxTotalBytes)
{
    if (!framed)
        Fail("directory verification requested outside a frame: " + std::string{key});
    const std::string directoryPath{path};
    const auto recorded = ProcessInputs(key, [directoryPath, maxTotalBytes] {
        return ReadAssetDirectory(directoryPath, maxTotalBytes);
    });
    if (Active() && recorded != ReadAssetDirectory(directoryPath, maxTotalBytes))
        Fail("deploy asset directory differs from recording: " + directoryPath);
}
namespace {
void VerifyDirectoryManifest(std::string_view key, std::string_view path, std::size_t maxTotalBytes)
{
    if (!framed)
        Fail("directory manifest verification requested outside a frame: " + std::string{key});
    const std::string directoryPath{path};
    const auto recorded = ProcessInputs(key, [directoryPath, maxTotalBytes] {
        return ReadAssetDirectoryManifest(directoryPath, maxTotalBytes);
    });
    if (Active() && recorded != ReadAssetDirectoryManifest(directoryPath, maxTotalBytes))
        Fail("deploy asset directory differs from recording: " + directoryPath);
}
} // namespace
void VerifyPathPlannerAssets()
{
    if (pathPlannerAssetsVerified)
        return;
    if (!framed) {
        if (active)
            Fail("PathPlanner assets must be verified inside a frame");
        return;
    }
    const auto deploy = frc::filesystem::GetDeployDirectory();
    VerifyDirectoryManifest("Framework/PathPlanner/DeployAssets/ManifestV1", deploy + "/pathplanner", 8 * 1024 * 1024);
    VerifyDirectoryManifest("Framework/Choreo/DeployAssets/ManifestV1", deploy + "/choreo", 64 * 1024 * 1024);
    pathPlannerAssetsVerified = true;
}
Json RecordedInput(std::string_view key)
{
    if (!framed)
        Fail("recorded input requested outside a frame: " + std::string(key));
    const auto name = std::string(key);
    if (!frame["inputs"].contains(name))
        Fail("missing required inputs: " + name);
    return frame["inputs"].at(name);
}
std::optional<Json> RecordedInputIfPresent(std::string_view key)
{
    if (!framed)
        Fail("recorded input requested outside a frame: " + std::string{key});
    const auto name = std::string{key};
    if (!frame["inputs"].contains(name))
        return std::nullopt;
    return frame["inputs"].at(name);
}
void RecordOutput(std::string_view key, const Json &value)
{
    if (!framed)
        return;
    Validate(value);
    frame["outputs"][std::string(key)] = value;
}
void EndFrame()
{
    if (!framed)
        return;
    const auto timestamp = frame.at("timestampUs").get<int64_t>();
    // Keep the per-cycle replay envelope available in structured WPILOG entries
    // for AdvantageScope and standard WPILib log inspection. Replay/Frames
    // remains the authoritative input schema used by the desktop runner.
    recorder->Submit(std::move(frame), timestamp);
    framed = false;
    if (active)
        ++cursor;
    else
        ++index;
    if (!active && index % 50 == 0)
    {
        const auto stats = recorder->Statistics();
        recordingStatus->Append(Json{{"written", stats.written},
                                     {"rejected", stats.rejected},
                                     {"failed", stats.failed},
                                     {"highWater", stats.highWater}}
                                    .dump(),
                                timestamp);
    }
}
void RunFrame(const std::function<void()> &callback)
{
    const bool ownsFrame = !FrameActive();
    if (ownsFrame)
        BeginFrame();

    std::exception_ptr callbackError;
    try
    {
        callback();
    }
    catch (...)
    {
        callbackError = std::current_exception();
    }

    if (ownsFrame)
    {
        try
        {
            EndFrame();
        }
        catch (...)
        {
            if (!callbackError)
                throw;
        }
    }

    if (callbackError)
        std::rethrow_exception(callbackError);
}
namespace
{
void PersistMarker(std::string_view entryName)
{
    if (recorder)
        recorder->Flush();
    const auto status = recorder ? recorder->Statistics() : finalRecordingStats;
    if (status.rejected || status.failed)
        Fail("recording worker rejected or failed a frame");
    const std::string marker = "replay-persisted-" + std::to_string(cursor) + "-" +
                               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    wpi::log::StringLogEntry completion(base::logging::GetLog(), "/" + std::string(entryName),
                                        base::logging::kAdvantageKitMetadata);
    completion.Append(marker);
    base::logging::GetLog().Flush(); // Nonblocking background-writer request.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do
    {
        std::ifstream file(outputPath, std::ios::binary);
        if (file)
        {
            file.seekg(0, std::ios::end);
            const auto end = file.tellg();
            file.seekg(std::max<std::streamoff>(0, static_cast<std::streamoff>(end) - 65536));
            std::string tail((std::istreambuf_iterator<char>(file)), {});
            if (tail.find(marker) != std::string::npos)
            {
                pendingReplayBytes = 0;
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    Fail("output writer did not persist replay marker within five seconds");
}
} // namespace
void FlushOutput()
{
    if (active)
        PersistMarker("Replay/Complete");
}
void ShutdownRecording()
{
    if (!recorder)
        return;
    recorder->Stop();
    const auto status = recorder->Statistics();
    finalRecordingStats = status;
    recordingStatus->Append(Json{
        {"written", status.written},
        {"rejected", status.rejected},
        {"failed", status.failed},
        {"highWater", status.highWater},
        {"final", true}}.dump());
    recorder.reset();
    base::logging::GetLog().Flush();
}
void RunReplay(const std::function<void()> &robotInit, const std::function<void()> &loop)
{
#ifndef __FRC_ROBORIO__
    if (!active)
        Fail("runner requires replay input");
    frc::sim::PauseTiming();
    frc::sim::RestartTiming();
    robotInit();
    int64_t last = 0;
    while (cursor < source.size())
    {
        auto now = source.at(cursor).at("timestampUs").get<int64_t>();
        // TimedRobot's notifier exists but its normal notifier loop is bypassed.
        // Synchronous StepTiming waits for that unconsumed notifier indefinitely;
        // this runner owns callback ordering and invokes LoopFunc directly below.
        frc::sim::StepTimingAsync(units::second_t{(now - last) * 1e-6});
        last = now;
        BeginFrame();
        InstallDS(frame.at("ds"));
        loop();
        EndFrame();
        // Offline replay can generate many seconds of data before the writer's
        // wall-clock flush period elapses. Apply backpressure BETWEEN callbacks;
        // live 20 ms controls never wait for disk and no replay frames are coalesced.
        if (cursor % 32 == 0 || pendingReplayBytes.load(std::memory_order_relaxed) >= 65536)
            PersistMarker("Replay/Progress");
    }
    std::cout << "[Replay] Completed " << cursor << " frames" << std::endl;
#else
    Fail("desktop-only runner");
#endif
}
} // namespace base::replay
