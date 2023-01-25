package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  static DriveSubsystem drivetrain;
  static CommandXboxController joystick;

  public DriveCommand(DriveSubsystem d, CommandXboxController j) {
    drivetrain = d;
    joystick = j;
    addRequirements(d);
  }

  @Override
  public void execute() {
    double x = DeadzoneCheck(joystick.getLeftY());
    double y = -DeadzoneCheck(joystick.getLeftX());
    double r = -DeadzoneCheck(joystick.getRightX());
    drivetrain.drive(x, y, r);
  }

  double DeadzoneCheck(double d) {
    if (Math.abs(d) < Operator.DeadzoneMin) d = 0;
    return d;
  }
}
