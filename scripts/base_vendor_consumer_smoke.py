#!/usr/bin/env python3
"""Link a clean robot against the Base release under validation.

The PathPlanner candidate workflow retains its published-Base smoke fixture in
vendor_consumer_smoke.py, so it does not require APIs from an unreleased Base.
Keep the common consumer setup aligned when changing either fixture.
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", choices=("roborio", "desktop"), required=True)
    parser.add_argument("--maven-repo", required=True)
    parser.add_argument("--descriptor", type=Path, help="Override vendordep descriptor for a staged package")
    parser.add_argument("--configuration", choices=("Debug", "Release"), default="Debug")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="meco-vendor-consumer-") as temporary:
        project = Path(temporary)
        (project / "vendordeps").mkdir()
        (project / "src/main/cpp").mkdir(parents=True)
        shutil.copy2(args.descriptor or repo / "vendor/MecoBase.json", project / "vendordeps/MecoBase.json")
        shutil.copy2(repo / "settings.gradle", project / "settings.gradle")
        (project / "src/main/cpp/Robot.cpp").write_text(
            '#include "base/Drive.h"\n'
            '#include "base/DriverInput.h"\n'
            '#include "base/DriverStationMatchInfo.h"\n'
            '#include "base/Replay.h"\n'
            '#include "base/CanBusMode.h"\n'
            '#include "base/AutoDrive.h"\n'
            '#include "base/FootprintAutoDrive.h"\n'
            '#include "base/FootprintSpeedProfile.h"\n'
            '#include "base/FootprintTerrainMap.h"\n'
            '#include "base/NavigationGrid.h"\n'
            '#include "base/ObjectPerception.h"\n'
            '#include "base/VisionIOSelector.h"\n'
            '#include "base/Mechanism.h"\n'
            '#include "base/SystemCheck.h"\n'
            'frc2::CommandPtr MakeAutoDrive(base::Drive& drive) { '
            'return base::AutoDriveToPose(drive, [] { return std::optional{frc::Pose2d{}}; }); }\n'
            'frc2::CommandPtr MakeTrackedAutoDrive(base::Drive& drive) { '
            'return base::AutoDriveTrackTarget(drive, [] { return std::optional{frc::Pose2d{}}; }); }\n'
            'frc2::CommandPtr MakeFootprintAutoDrive(base::Drive& drive) { '
            'base::FootprintAutoDriveConfig config; config.footprint = {.7, .8}; '
            'config.field = {0, 0, 16, 8}; '
            'return base::AutoDriveFootprintToPose(drive, [] { return std::optional{frc::Pose2d{}}; }, '
            '[] { return std::vector<base::FieldRectangle>{}; }, config); }\n'
            'base::ObjectPerception MakePerception() { '
            'std::vector<base::ObjectCamera> cameras; '
            'cameras.push_back({"front", base::MakeNtObjectDetectionIO("/camera"), '
            '[] { return std::optional{frc::Transform3d{}}; }}); '
            'return base::ObjectPerception(std::move(cameras), [] { return frc::Pose3d{}; }); }\n'
            'void ExerciseRuntimeMotorFactory(const base::MotorConfig& config) { '
            'base::MotorIOSelector selector; '
            'auto motor=selector.Make(base::Vendor::SparkMax, config); '
            'double rate=base::SelectCanBusRate(config.canBus, 100., 50., false); (void)rate; }\n'
            'void ExerciseRuntimeVisionFactories() { '
            'auto a=base::MakeRuntimeLimelightVisionIO({}); '
            'auto b=base::MakeRuntimeQuestNavVisionIO({}); '
            'auto c=base::MakeRuntimeObjectDetectionIO("/camera", units::second_t{.1}); }\n'
            'base::DriverInputState ExerciseDriverInput(frc::XboxController& controller, base::NtGamepad* nt) { '
            'return base::ReadDriverInput(controller, nt, true); }\n'
            "int main() { auto match=base::ReadDriverStationMatchInfo(); (void)match; "
            "base::DriveConfig{}.Validate(); base::Gains{}.Validate(false); "
            "base::SystemCheck check([] {}); check.Cancel(); "
            "base::FootprintTerrainMap terrain({{{1,0,2,2},.1,0,.01}}); "
            "auto cuts=terrain.Boundaries({0,1,0},{3,1,0}); "
            "auto speed=base::FootprintSegmentSpeed(1., .5, 0., 0., {}, {}); "
            "base::NavigationGrid grid{1., {{false}}}; "
            "bool free=base::IsNavigationGridPointFree(grid, frc::Translation2d{}); "
            "auto point=base::FindReachableGridPoint(grid, frc::Translation2d{}, [](auto) { return true; }); "
            "auto loaded=base::LoadNavigationGrid(\"missing-navgrid.json\"); "
            'auto parsed=base::ParseNavigationGrid(R"({"nodeSizeMeters":1.0,"grid":[[false]]})"); '
            "base::replay::VerifyPathPlannerAssets(); "
            "return cuts.size()==2 && speed && free && point && !loaded && parsed && base::ValidAutoDriveTarget(frc::Pose2d{}, base::AutoDriveConfig{}) ? 0 : 1; }\n",
            encoding="utf-8",
        )
        platform = "wpi.platforms.roborio" if args.target == "roborio" else "wpi.platforms.desktop"
        (project / "build.gradle").write_text(
            f'''plugins {{ id "c"; id "cpp"; id "edu.wpi.first.GradleRIO" version "2026.2.1" }}
repositories {{ maven {{ url = uri(findProperty("mecoVendorRepo")) }} }}
def vendorCache = providers.environmentVariable('FRC_VENDOR_CACHE').orNull
if (vendorCache) {{
    repositories {{
        exclusiveContent {{
            forRepository {{ maven {{ url = uri(file(vendorCache)); metadataSources {{ mavenPom(); artifact() }} }} }}
            filter {{ includeGroup('com.studica.frc') }}
        }}
    }}
}}
nativeUtils.platformConfigs.all {{
    cppCompiler.args.removeAll {{ it.startsWith('-std=') || it.startsWith('/std:') }}
    cppCompiler.args.add(name.startsWith('windows') ? '/std:c++latest' : '-std=c++23')
}}
model {{ components {{ frcUserProgram(NativeExecutableSpec) {{
    targetPlatform {platform}
    sources.cpp {{
        source {{ srcDir 'src/main/cpp'; include '**/*.cpp' }}
    }}
    wpi.cpp.vendor.cpp(it)
    wpi.cpp.deps.wpilib(it)
}} }} }}
''',
            encoding="utf-8",
        )

        gradle = repo / ("gradlew.bat" if os.name == "nt" else "gradlew")
        maven_repo = Path(args.maven_repo).resolve() if "://" not in args.maven_repo else args.maven_repo
        command = [str(gradle), "-p", str(project),
                   f"linkFrcUserProgram{args.configuration}Executable",
                   f"-PmecoVendorRepo={maven_repo}", "--max-workers=1"]
        if os.name == "nt":
            command.extend(["-Ptoolchain-optional-roboRio"])
            command = ["cmd", "/c", *command]
        subprocess.run(command, cwd=repo, check=True)


if __name__ == "__main__":
    main()
