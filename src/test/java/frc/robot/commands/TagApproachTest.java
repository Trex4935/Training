package frc.robot.commands;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.*;
class TagApproachTest {
 @Test void standoffUsesTagLocalAxisAndRobotClearance() {
   Pose2d tag = new Pose2d(4, 5, Rotation2d.fromDegrees(90));
   Pose2d goal = TagApproach.target(tag, .3, .4, .2);
   assertEquals(4, goal.getX(), 1e-9);
   assertEquals(4.4, goal.getY(), 1e-9);
   assertEquals(90, goal.getRotation().getDegrees(), 1e-9);
 }
 @Test void honorsLargerConfiguredStandoff() {
   assertEquals(1.0, TagApproach.target(new Pose2d(3,2,Rotation2d.kZero),2,.4,.1).getX(),1e-9);
 }
}
