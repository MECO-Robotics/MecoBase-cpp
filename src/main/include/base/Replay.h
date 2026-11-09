#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <wpi/json.h>
namespace base::replay
{
// Desktop FRC_REPLAY_LOG enables input substitution. Active() initializes a
// requested replay before IO selection; live logging begins on explicit initialization.
// Initialize early when replay frame processing or output logging is required.
void InitializeFromEnvironment();
bool Active();
bool FrameActive();
uint64_t FrameIndex();
wpi::json ProcessInputs(std::string_view key, std::function<wpi::json()> live);
// Capture a bounded text asset at its first behavior boundary and return the
// recorded bytes during replay. Missing files are represented as nullopt.
// These asset APIs require an active frame so setup cannot consume a cycle.
std::optional<std::string> ProcessFileInput(std::string_view key, std::string_view path,
                                           std::size_t maxBytes);
// Require a local asset to match the bytes captured in the active replay frame.
// Use when a third-party parser only accepts a filesystem path.
void VerifyFileInput(std::string_view key, std::string_view path, std::size_t maxBytes);
// Capture all regular files under a deploy directory once and fail replay if
// the local asset tree differs from the recording.
void VerifyDirectoryInput(std::string_view key, std::string_view path, std::size_t maxTotalBytes);
// Record bounded manifests of standard deploy/pathplanner and deploy/choreo
// trees. This guards third-party path loaders that accept names but do not
// accept recorded bytes; it does not reconstruct those loaders from the log.
void VerifyPathPlannerAssets();
void RecordOutput(std::string_view key, const wpi::json &value);
template <typename T, typename Reader>
T ProcessValue(std::string_view key, Reader &&live)
{
    auto value = ProcessInputs(key, [&live] { return wpi::json(live()); });
    RecordOutput(std::string{"EffectiveInputs/"} + std::string{key}, value);
    return value.template get<T>();
}
// Read a required value already present in the active frame's input snapshot.
// Replay fails closed when the caller asks for an input that was not recorded.
wpi::json RecordedInput(std::string_view key);
// Read a recorded value when an adapter supplied it, without requiring it in
// every input schema. The caller must still be inside an active frame.
std::optional<wpi::json> RecordedInputIfPresent(std::string_view key);
void BeginFrame();
void EndFrame();
// Execute one robot callback within a frame. Nested callers reuse the active
// frame; the outermost caller always closes its frame, including on exceptions.
void RunFrame(const std::function<void()> &callback);
// EOF only: request the asynchronous writer flush and wait for its completion
// marker on disk before stopping the logger. Never called in the robot loop.
void FlushOutput();
// Drain/join the recording worker before stopping DataLogManager. Lifecycle only.
void ShutdownRecording();
// Runs existing robot callbacks once per recorded timestamp, installing HAL DS
// data and FPGA time first. Returns only after EOF; no physics is constructed.
void RunReplay(const std::function<void()> &robotInit, const std::function<void()> &loop);
} // namespace base::replay
