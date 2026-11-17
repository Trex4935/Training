# Hallway vision map setup

The training target is Tag 28. Java loads [hallway-28.json](../src/main/deploy/hallway-28.json); the Limelight continues using the `.fmap` uploaded to the camera. Both files must come from the same source map.

## Install the hallway map

1. In the Limelight web UI, save or back up the current competition `.fmap`.
2. Upload the hallway `fiducials.fmap` to the Limelight 4 CM5 and confirm Tag 28 is detected.
3. Convert that exact file for Java:

   ```powershell
   .\gradlew.bat convertLimelightFmap "-Pfmap=C:\path\to\fiducials.fmap" "-PfmapOutput=src/main/deploy/hallway-28.json"
   ```

4. Confirm the generated layout contains Tag 28 and matches the camera map. The converter keeps every tag in the `.fmap`, preserves its height and 3D orientation, converts the centered FMAP coordinates to WPILib's blue-origin coordinates using the map's field dimensions, and rejects malformed inputs.
5. Deploy the robot code. If the Java layout is missing or does not include ID 28, field-relative tag commands return no target and stop.
6. Configure the tag-relative POI, robot dimensions, and clearance in `RobotContainer`. These offsets are local to Tag 28; they are not absolute field positions.

## Restore competition operation

Upload the backed-up competition `.fmap` to the Limelight, then regenerate the Java `AprilTagFieldLayout` JSON from that same competition map before using field-relative tag commands. Keep the training JSON or regenerate only after choosing the matching map. Do not mix a camera map and Java layout from different fields.

The Java converter task is `convertLimelightFmap`; it does not upload maps to the Limelight. Verify camera pose, localization, target direction, and clearances at low speed with a spotter before driving on the floor. These commands do not provide obstacle avoidance.
