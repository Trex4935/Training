package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

/** Slow, tag-relative approach. A recent sighting and current field pose are both required. */
public class DriveUnderTag extends Command {
  private final Limelight vision;
  private final int id;
  private final DriveToTagPose controller;
  public DriveUnderTag(Limelight vision,CommandSwerveDrivetrain drive,TagApproach.Config target){
    this.vision=vision; this.id=target.tagId();
    controller=new DriveToTagPose(drive,()->TagApproach.target(vision.getTagFieldPose3d(id),target),
        ()->vision.hasTrustedFieldPose() && vision.hasTag(id));
    addRequirements(drive);
  }
  @Override public void initialize(){controller.initialize();}
  @Override public void execute(){controller.execute();}
  @Override public boolean isFinished(){return controller.isFinished();}
  @Override public void end(boolean interrupted){controller.end(interrupted);}
}
