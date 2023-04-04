package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;

public class AutonomousCommand extends CommandBase {
  private DriveSubsystem drivetrain;
  private ArmSubsystem arm;
  private ClawSubsystem claw;
  private InformationSubsystem info;

  public AutonomousCommand(
      DriveSubsystem drive, ArmSubsystem arm, ClawSubsystem claw, InformationSubsystem info) {
    this.drivetrain = drive;
    this.arm = arm;
    this.claw = claw;
    this.info = info;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    info.driveUntil(drivetrain, new Pose2d(new Translation2d(0, 0), new Rotation2d()), 1);
    // info.driveUntil(drivetrain, new Pose2d(new Translation2d(0, -0.2), new Rotation2d()), 1);
    // info.driveUntil(drivetrain, new Pose2d(new Translation2d(0.2, 0), new Rotation2d()), 1);
    // info.driveUntil(drivetrain, new Pose2d(new Translation2d(0, 0.2), new Rotation2d()), 1);
  }
}
