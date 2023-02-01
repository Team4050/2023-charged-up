package frc.robot.commands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DanceCommand extends CommandBase {
  private final DriveSubsystem drive;

  public DanceCommand(DriveSubsystem subsystem) {
    drive = subsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.orchestra.loadMusic(Filesystem.getDeployDirectory().getAbsolutePath() + "/rave.chrp");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.orchestra.play();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.orchestra.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !drive.orchestra.isPlaying();
  }
}
