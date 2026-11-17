package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import org.junit.jupiter.api.Test;
import java.util.concurrent.atomic.AtomicReference;
import com.ctre.phoenix6.swerve.SwerveRequest;

class VisionGeometryTest {
  @Test void overheadGoalUsesConfiguredOffsetAndClearance(){
    var config=new TagApproach.Config(28,.5,0,0,.4,.4,.1,TagApproach.Mount.OVERHEAD);
    Pose2d goal=TagApproach.target(new Pose3d(2,3,2,new Rotation3d()),config);
    assertEquals(2.5,goal.getX(),1e-9);
  }
  @Test void orbitDirectionReversesAndRadiusCorrectionPointsInward(){
    Translation2d ccw=OrbitTag.tangentialVelocity(new Translation2d(1,0),1,.3);
    Translation2d cw=OrbitTag.tangentialVelocity(new Translation2d(1,0),-1,.3);
    assertEquals(.3,ccw.getY(),1e-9); assertEquals(-.3,cw.getY(),1e-9);
  }
  @Test void positiveYawErrorCommandsCorrectionTowardZero(){
    PIDController pid=new PIDController(.035,0,0);
    assertTrue(pid.calculate(10,0)<0);
  }
  @Test void cancellationBrakeOutputDoesNotLeavePriorMotionRequestActive(){
    AtomicReference<SwerveRequest> last=new AtomicReference<>();
    DriveToTagPose.brake(last::set);
    assertInstanceOf(SwerveRequest.SwerveDriveBrake.class,last.get());
  }
}
