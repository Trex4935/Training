# Vision commands for students

`Limelight` owns camera details: MegaTag1 heading bootstrap, MegaTag2 pose measurements, filtering, IMU modes, tag observations, timestamps, and access to tag field poses. Commands use its small API and never need `LimelightHelpers`.

Field poses use blue-origin coordinates: X forward, Y left, and positive heading counterclockwise. Camera `tx` is a horizontal angle in degrees; it is useful for aiming but does not tell you a reliable field distance. Vision observations expire after 0.25 s. Field navigation additionally needs a recently accepted MegaTag2 pose.

An AprilTag's **field pose** is its absolute position and orientation in the field map. A **POI** is where the robot should be relative to that tag. `TagApproach.Config` holds the selected tag ID, tag-local forward/left offsets, relative robot heading, robot length/width, clearance, and mount type. The tag's orientation rotates those offsets into field coordinates; Java does not store tag field X/Y values.

## Controls

- Left stick translates; right stick rotates manually.
- LB sets driver-forward perspective only. Back restores standard perspective.
- Hold RB to rotate toward explicitly selected training tag ID 28 while keeping stick translation.
- Hold A to drive to a conservative field pose relative to that tag.
- Hold X to orbit the selected tag-relative POI counterclockwise while maintaining the configured heading.
- Hold Y to approach the configured overhead POI. For wall-mounted tags, select the `WALL` mount and use a negative local-forward offset away from the tag face. Configure offsets and clearance in `RobotContainer` before use.

All behavior commands require the drivetrain. Releasing the button cancels the command, stops its output, and lets default manual driving resume. If target or localization data expires, automated commands stop. Speeds and tolerances are in `FaceTag`, `DriveToTagPose`, and `OrbitTag` for easy student tuning.

## Make a small vision command

1. Add a `Command` with the drivetrain as a requirement.
2. Ask `Limelight` for one measurement, such as a timestamp-checked yaw or tag field pose.
3. Check validity before using it. An invalid observation means stop.
4. Calculate a small error (target minus current measurement).
5. Convert the error to bounded drivetrain velocity with PID or simple geometry.
6. Stop in `end()` so release or interruption is safe.

Keep camera parsing and localization inside `Limelight`. Do not reset pose or heading from a student movement command. `TagApproach` transforms the POI from tag axes into field axes and projects its height onto the robot floor plane. It checks clearance using the robot's circumscribed radius. These commands do not avoid obstacles.

## Setup and hardware checks

The LL4's uploaded `.fmap` is authoritative and is not exposed as tag poses through NetworkTables. Java loads `src/main/deploy/hallway-28.json`, converted from the same `fiducials.fmap` source using `LimelightFmapConverter`. The conversion changes the centered field origin used by the FMAP into WPILib's blue-alliance origin using the source field dimensions, while preserving each tag's 3D rotation and height. Regenerate it with `./gradlew convertLimelightFmap -Pfmap=<source.fmap> -PfmapOutput=src/main/deploy/hallway-28.json` whenever the source map changes. If the JSON is absent, invalid, or does not contain ID 28, field-pose commands stop; no season-layout or coordinate fallback is used. Keep the original `.fmap` uploaded to the camera.

Before motion, confirm that the selected tag exists in both maps, the camera transform and field heading are correct, and the relative POI has safe clearance. Start on blocks or at very low speed with a spotter and a clear stopping area.

Automated hardware validation remains required: verify selected-tag visibility and freshness, field-layout alignment, MegaTag2 pose acceptance, driver perspective behavior, motion directions, target-loss stopping, cancellation/manual resumption, and wall/overhead offsets. A successful software build does not validate camera calibration or physical clearances.
