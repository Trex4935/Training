package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.*;
import org.junit.jupiter.api.Test;

class TagApproachTest {
  private final TagApproach.Config wall=new TagApproach.Config(28,-1.0,0.0,0.0,.8,.6,.1,TagApproach.Mount.WALL);
  @Test void changingMapPoseMovesRobotPoiBySameFieldDelta(){
    Pose2d a=TagApproach.target(new Pose3d(2,3,1,new Rotation3d()),wall);
    Pose2d b=TagApproach.target(new Pose3d(4,6,1,new Rotation3d()),wall);
    assertEquals(2,b.getX()-a.getX(),1e-9); assertEquals(3,b.getY()-a.getY(),1e-9);
  }
  @Test void sameLocalOffsetRotatesWithTag(){
    Pose2d a=TagApproach.target(new Pose3d(2,3,1,new Rotation3d()),wall);
    Pose2d b=TagApproach.target(new Pose3d(2,3,1,new Rotation3d(0,0,Math.PI/2)),wall);
    assertEquals(1,a.getX(),1e-9); assertEquals(2,b.getX(),1e-9); assertEquals(2,b.getY(),1e-9);
  }
  @Test void missingOrUnsafeMapPoseNeverFallsBackToCoordinates(){
    assertNull(TagApproach.target(null,wall));
    var tooClose=new TagApproach.Config(28,-.2,0,0,.8,.6,.1,TagApproach.Mount.WALL);
    assertNull(TagApproach.target(new Pose3d(100,100,0,new Rotation3d()),tooClose));
  }
  @Test void overheadPoseProjectsToFloorAndChecksRobotClearance(){
    var overhead=new TagApproach.Config(28,.8,.2,Math.PI/2,.8,.6,.1,TagApproach.Mount.OVERHEAD);
    Pose2d goal=TagApproach.target(new Pose3d(2,3,2,new Rotation3d(0,0,Math.PI/2)),overhead);
    assertEquals(1.8,goal.getX(),1e-9); assertEquals(3.8,goal.getY(),1e-9); assertEquals(Math.PI,goal.getRotation().getRadians(),1e-9);
  }
}
