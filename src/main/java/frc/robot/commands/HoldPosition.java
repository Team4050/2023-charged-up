package frc.robot.commands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;

public class HoldPosition extends CommandBase {
  private DriveSubsystem drive;
  private InformationSubsystem info;
  private Matrix<N3, N1> targetPose;

  private PIDController X;
  private PIDController Y;
  private PIDController Z;

  public HoldPosition(DriveSubsystem drive, Matrix<N3, N1> target) {
    this.drive = drive;
    targetPose = target;

    X = new PIDController(0.2, 0, 0);
    Y = new PIDController(0.2, 0, 0);
    Z = new PIDController(0.2, 0, 0);
  }

  public void execute() {
    /*drive.drive(
    X.calculate(info.getPoseEstimate().get(0, 0), targetPose.get(0, 0)),
    Y.calculate(info.getPoseEstimate().get(1, 0), targetPose.get(1, 0)),
    Z.calculate(info.getPoseEstimate().get(2, 0), targetPose.get(2, 0)));*/

    SmartDashboard.putNumberArray(
        "Station keeping PID",
        new double[] {
          X.calculate(info.getPoseEstimate().get(0, 0), targetPose.get(0, 0)),
          Y.calculate(info.getPoseEstimate().get(1, 0), targetPose.get(1, 0)),
          Z.calculate(info.getPoseEstimate().get(2, 0), targetPose.get(2, 0))
        });
  }
}
