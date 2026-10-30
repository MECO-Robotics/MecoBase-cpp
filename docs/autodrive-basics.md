# AutoDrive basics

Want the robot to drive to a point on the field? Give AutoDrive a target pose. Your robot code chooses when to run it and where to go.

![AutoDrive from target to drivetrain](visuals/autodrive-basics.png)

[Diagram source](visuals/autodrive-basics.mmd)

- Set up the native pathfinder before calling `Drive::ConfigureAuto()`.
- Use `AutoDriveToPose` for a fixed target. It reads the pose when the command starts.
- Use `AutoDriveTrackTarget` for a moving target. Your supplier must reject old observations.
- Use blue-origin field coordinates for either alliance; the command does not flip them.
- Deploy a navgrid with enough clearance for your robot. Check localization and limits before driving.
- A missing or invalid target stops the robot. Interruption or disable stops it too.

Read the [full AutoDrive contract](autodrive.md), [public interface](../src/main/include/base/AutoDrive.h), and [implementation](../src/main/cpp/base/drive/AutoDrive.cpp). A season project owns its controller bindings, target selection, and game strategy.
