#!/usr/bin/env python3
"""Link a tiny clean robot project against the staged MECO vendordep."""

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
    parser.add_argument("--configuration", choices=("Debug", "Release"), default="Debug")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="meco-vendor-consumer-") as temporary:
        project = Path(temporary)
        (project / "vendordeps").mkdir()
        (project / "src/main/cpp").mkdir(parents=True)
        shutil.copy2(repo / "vendor/MecoBase.json", project / "vendordeps/MecoBase.json")
        shutil.copy2(repo / "settings.gradle", project / "settings.gradle")
        (project / "src/main/cpp/Robot.cpp").write_text(
            '#include "base/Drive.h"\n'
            '#include "base/AutoDrive.h"\n'
            '#include "base/FootprintAutoDrive.h"\n'
            '#include "base/FootprintSpeedProfile.h"\n'
            '#include "base/ObjectPerception.h"\n'
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
            "int main() { base::DriveConfig{}.Validate(); base::Gains{}.Validate(false); "
            "base::SystemCheck check([] {}); check.Cancel(); "
            "auto speed=base::FootprintSegmentSpeed(1., .5, 0., 0., {}, {}); "
            "return speed && base::ValidAutoDriveTarget(frc::Pose2d{}, base::AutoDriveConfig{}) ? 0 : 1; }\n",
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
