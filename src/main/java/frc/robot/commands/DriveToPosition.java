package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import java.util.ArrayList;

public class DriveToPosition extends CommandBase {
  private DriveSubsystem drive;
  private InformationSubsystem info;
  private Pose2d targetPose;
  private Trajectory trajectory;
  private PIDController X;
  private PIDController Y;
  private PIDController Rotation;
  private Timer timer;

  public DriveToPosition(DriveSubsystem drive, InformationSubsystem info, ArrayList<Pose2d> curve) {
    this.drive = drive;
    this.info = info;
    targetPose = curve.get(curve.size() - 1);
    trajectory = TrajectoryGenerator.generateTrajectory(curve, new TrajectoryConfig(10, 2));
  }

  @Override
  public void execute() {
    drive.driveFieldRelative(
        X.calculate(
            info.getPoseEstimate().getX() - trajectory.sample(timer.get()).poseMeters.getX()),
        Y.calculate(
            info.getPoseEstimate().getY() - trajectory.sample(timer.get()).poseMeters.getY()),
        /* Very complicated function string, basically just take the current robot rotation,
         * rotate that by the inverse of the target rotation, and measure that rotation's degree to get the error value.
         */
        Rotation.calculate(
            info.getPoseEstimate()
                .toPose2d()
                .getRotation()
                .rotateBy(trajectory.sample(timer.get()).poseMeters.getRotation().unaryMinus())
                .getRadians()));
  }

  @Override
  public void initialize() {
    X = new PIDController(0.3, 0.1, 0.2);
    Y = new PIDController(0.3, 0.1, 0.2);
    Rotation = new PIDController(0.3, 0.1, 0.2);
    X.setSetpoint(0);
    Y.setSetpoint(0);
    Rotation.setSetpoint(0);
    timer = new Timer();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds() + 0.1;
  }
}
