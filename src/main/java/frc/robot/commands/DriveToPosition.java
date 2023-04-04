package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

public class DriveToPosition extends CommandBase {
  private DriveSubsystem drive;
  private InformationSubsystem info;
  private Trajectory trajectory;
  private PIDController X;
  private PIDController Y;
  private ProfiledPIDController Rotation;
  private HolonomicDriveController controller;
  private Timer timer = new Timer();
  private Set<Subsystem> reqs;

  public DriveToPosition(DriveSubsystem drive, InformationSubsystem info, ArrayList<Pose2d> curve) {
    this.drive = drive;
    this.info = info;
    trajectory = TrajectoryGenerator.generateTrajectory(curve, new TrajectoryConfig(0.1, 0.2));
    HashSet<Subsystem> set = new HashSet<Subsystem>();
    set.add(drive);
    reqs = set;
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds =
        controller.calculate(
            info.getPoseEstimate().toPose2d(),
            trajectory.sample(timer.get()),
            trajectory.sample(timer.get()).poseMeters.getRotation());
    // System.out.println(trajectory.sample(timer.get()).poseMeters);
    System.out.println(
        String.format(
            "%f, %f, %f",
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond));
    drive.driveFieldRelative(
        speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
  }

  @Override
  public void initialize() {
    X = new PIDController(0.1, 0, 0);
    Y = new PIDController(0.1, 0, 0);
    Rotation = new ProfiledPIDController(0.1, 0, 0, new Constraints(1, 1));
    controller = new HolonomicDriveController(X, Y, Rotation);
    // Tolerance of +-5cm & +-5 degrees
    controller.setTolerance(new Pose2d(0.05, 0.05, new Rotation2d(Math.PI / 72)));
    timer = new Timer();
    timer.start();
    // drive.setMaxOutput(0.2);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return reqs;
  }

  @Override
  public boolean isFinished() {
    return timer.get() > (trajectory.getTotalTimeSeconds() + 0.1);
  }
}
