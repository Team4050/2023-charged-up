package frc.robot.commands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;

public class DriveToPosition extends CommandBase {
  private DriveSubsystem drive;
  private InformationSubsystem info;
  private Matrix<N3, N1> targetState;

  public DriveToPosition(DriveSubsystem drive, InformationSubsystem info, Matrix<N3, N1> target) {
    this.drive = drive;
    this.info = info;
    targetState = target;
  }

  @Override
  public void execute() {}

  @Override
  public void initialize() {}
}
