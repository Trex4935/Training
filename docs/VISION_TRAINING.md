# Vision and driver heading

The swerve drivetrain continuously estimates motion from its gyro and wheel encoders. Limelight MegaTag2 uses AprilTags plus the robot's current field heading to estimate field X/Y. The timestamp lets CTRE apply a camera frame at its capture time, then replay the later odometry. Filtering avoids stale, duplicate, non-finite, and implausibly distant frames.

Read `CommandSwerveDrivetrain`, `Limelight`, then `RobotContainer`. The pose shown by CTRE telemetry is the fused global estimate; Limelight pose keys show the latest accepted camera pose.

## Controls

- Left bumper calibrates the driver's forward direction to the robot's current estimated heading. It changes operator perspective only; it does not reset global pose or gyro calibration.
- Back returns to blue/standard perspective. The drivetrain's normal disabled-periodic alliance code may apply red perspective when disabled and the DS reports red.
- Hold right bumper to rotate toward the visible tag in `limelight-bow`; translation remains on the sticks. If the target disappears, rotation output is zero and the default manual command resumes on release.

## Hallway setup and testing

Configure Limelight name and physical camera mounting (forward-facing `limelight-bow`) in `RobotContainer`/`Limelight`. Configure the Limelight's own AprilTag map with the actual hallway tag ID and measured field pose, using the blue-origin NWU convention (X forward, Y left, yaw counterclockwise). No competition field coordinates are included here. MegaTag2 needs a valid field heading; initially align the robot to the map's known heading or use a heading-capable bootstrap before trusting its position.

1. Enable with no tags and drive slowly; odometry should continue.
2. Confirm the camera sees the known tag, accepted measurements update pose, and X/Y/heading telemetry is finite.
3. At robot headings 0, 90, 180, and 270 degrees, press left bumper and check stick-forward drives away from the driver-defined rear reference as intended. Back restores standard perspective.
4. Hold right bumper while viewing a tag. Translation should remain responsive. If it turns away, reverse the yaw sign in `RobotContainer` after checking camera orientation.
5. Calibrate manual forward, then acquire a tag; vision should refine pose without changing driver perspective. Disable/re-enable and confirm alliance perspective behavior.

Tune the aiming PID with low proportional gain first. Increase until response is useful, then add a small derivative term only if it oscillates. The angular output is capped at 1.5 rad/s and has a 1.5 degree dead zone. A reversed turn means yaw sign or camera mounting convention is reversed.

Common issues: wrong NetworkTables camera name, unconfigured Limelight map, wrong robot-to-camera transform, stale NetworkTables frames, incorrect field heading supplied to MegaTag2, and inverted camera yaw. Validate map pose and camera transform on the actual robot before relying on localization. Build success is not physical validation.

## Hardware validation checklist

- Confirm each Limelight model and firmware. LL4 IMU modes stay disabled in this project until the camera is confirmed and `kHasLimelight4Imu` is intentionally enabled.
- Measure and configure each camera-to-robot transform (forward, left, height, roll, pitch, yaw) in the Limelight configuration.
- Configure the physical hallway AprilTag layout with the correct tag family, ID, field X/Y/height, and orientation. Do not use competition field coordinates.
- Test single-tag bootstrap from several stationary viewpoints. It requires low ambiguity and 12 consecutive fresh, stable frames.
- After initialization, compare MegaTag1 and MegaTag2 pose estimates before enabling sustained driving.
- Verify driver perspective before and after bootstrap; pose seeding must not change joystick-forward.
- Verify tag-lock rotation sign at low speed and confirm target loss stops automatic rotation immediately.

## Low-speed drive-to-tag exercise

With the robot on blocks or in a cleared, supervised hallway, first confirm the configured tag ID and pose. The A hold command uses tag 1's configured pose and approaches along the tag's local negative X axis. Its final center distance is the larger of the requested standoff and the robot front extent plus clearance. The sample speed limit is 0.6 m/s; lower it further for first tests. Confirm global heading has bootstrapped and the selected tag is visible before holding A. Keep an operator ready to release A; release or command interruption brakes the drivetrain. Test the target pose geometry on paper/simulation first, then use a spotter and a large clear stopping area. This command does not avoid obstacles.
