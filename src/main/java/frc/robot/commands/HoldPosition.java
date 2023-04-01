package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import java.util.HashSet;
import java.util.Set;

public class HoldPosition extends CommandBase {
  private DriveSubsystem drive;
  private InformationSubsystem info;
  private Pose2d targetPose;

  private Set<Subsystem> requirements;

  private HazardXbox primaryControl;

  private PIDController X;
  private PIDController Y;
  private ProfiledPIDController Z;
  private HolonomicDriveController controller;

  public HoldPosition(
      DriveSubsystem drive, InformationSubsystem info, Pose2d target, HazardXbox controller) {
    this.primaryControl = controller;
    this.drive = drive;
    this.info = info;
    targetPose = target;

    requirements = new HashSet<Subsystem>();
    requirements.add(drive);

    X = new PIDController(0.2, 0.2, 0);
    Y = new PIDController(0.2, 0.2, 0);
    Z = new ProfiledPIDController(0.2, 0.2, 0, new Constraints(1, 1));
    this.controller = new HolonomicDriveController(X, Y, Z);
  }

  public void execute() {
    ChassisSpeeds speeds =
        controller.calculate(
            info.getPoseEstimate().toPose2d(), targetPose, 0, targetPose.getRotation());
    drive.driveFieldRelative(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }
}
