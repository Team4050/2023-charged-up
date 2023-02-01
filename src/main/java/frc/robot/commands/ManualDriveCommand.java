package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.DriveSubsystem;

public class ManualDriveCommand extends CommandBase {

  private static DriveSubsystem drivetrain;
  private static CommandXboxController joystick;

  public ManualDriveCommand(DriveSubsystem d, CommandXboxController j) {
    drivetrain = d;
    joystick = j;
    addRequirements(d);
  }

  @Override
  public void execute() {
    double x = deadzoneCheck(joystick.getLeftY());
    double y = -deadzoneCheck(joystick.getLeftX());
    double r = -deadzoneCheck(joystick.getRightX());

    drivetrain.drive(x, y, r);
    drivetrain.go();
  }

  private double deadzoneCheck(double d) {
    if (Math.abs(d) < Operator.DeadzoneMin) d = 0;
    return d;
  }
}
