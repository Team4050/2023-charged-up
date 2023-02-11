package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Operator;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.DriveSubsystem;

public class ManualDriveCommand extends CommandBase {

  private static DriveSubsystem drivetrain;
  private static HazardXbox joystick;

  public ManualDriveCommand(DriveSubsystem d, HazardXbox j) {
    drivetrain = d;
    joystick = j;
    addRequirements(d);
  }

  @Override
  public void execute() {
    drivetrain.drive(joystick.getLeftY(), joystick.getLeftX(), joystick.getRightX());
    drivetrain.go();
  }

  private double deadzoneCheck(double d) {
    if (Math.abs(d) < Operator.DeadzoneMin) d = 0;
    return d;
  }
}