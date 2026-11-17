package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class BootstrapStabilityTest {
  private final BootstrapStability gate = new BootstrapStability();
  private final Pose2d pose = new Pose2d(2, 3, Rotation2d.fromDegrees(25));

  @Test void multiTagNeedsFiveFreshStableFrames() {
    for (int i = 1; i <= 5; i++) assertEquals(i, gate.observe(pose, i * .02, i * .02, 2, .5, 2, 0, 0));
    assertEquals(5, gate.requiredFrames(2));
  }

  @Test void singleTagNeedsTwelveLowAmbiguityFrames() {
    for (int i = 1; i <= 12; i++) gate.observe(pose, i * .02, i * .02, 1, .05, 2, 0, 0);
    assertEquals(12, gate.requiredFrames(1));
    gate.reset();
    assertEquals(0, gate.observe(pose, .02, .02, 1, .2, 2, 0, 0));
  }

  @Test void rejectsStaleDuplicateMovingDistantAndUnstableFrames() {
    assertEquals(0, gate.observe(pose, .01, 1, 2, 0, 1, 0, 0));
    assertEquals(1, gate.observe(pose, .02, .02, 2, 0, 1, 0, 0));
    assertEquals(0, gate.observe(pose, .02, .03, 2, 0, 1, 0, 0));
    assertEquals(0, gate.observe(pose, .04, .04, 2, 0, 1, .5, 0));
    assertEquals(1, gate.observe(pose, .06, .06, 2, 0, 1, 0, 0));
    assertEquals(1, gate.observe(new Pose2d(3, 3, Rotation2d.kZero), .08, .08, 2, 0, 1, 0, 0));
  }
}
