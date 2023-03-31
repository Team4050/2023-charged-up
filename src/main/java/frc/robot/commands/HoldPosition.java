package frc.robot.commands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
  private Matrix<N3, N1> targetPose;

  private Set<Subsystem> requirements;

  private HazardXbox primaryControl;

  private PIDController X;
  private PIDController Y;
  private PIDController Z;

  public HoldPosition(
      DriveSubsystem drive,
      InformationSubsystem info,
      Matrix<N3, N1> target,
      HazardXbox controller) {
    this.primaryControl = controller;
    this.drive = drive;
    this.info = info;
    targetPose = target;

    requirements = new HashSet<Subsystem>();
    requirements.add(drive);

    X = new PIDController(0.2, 0.2, 0);
    Y = new PIDController(0.2, 0.2, 0);
    Z = new PIDController(0.2, 0.2, 0);
  }

  public void execute() {
    double[] array =
        new double[] {
          X.calculate(info.getPoseEstimate().getX(), targetPose.get(0, 0)),
          Y.calculate(info.getPoseEstimate().getY(), targetPose.get(1, 0)),
          Z.calculate(info.getPoseEstimate().getRotation().getZ(), targetPose.get(2, 0))
        };

    drive.driveSmart(
        primaryControl.getLeftY(), -primaryControl.getLeftX(), -primaryControl.getRightX());
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }
}
