package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import java.util.HashSet;
import java.util.Set;

public class AutonomousStep extends CommandBase {
  /* Drive subsystem & information stuff */
  private DriveSubsystem drive;
  private InformationSubsystem info;
  private Pose2d dest;

  /* PID control */
  private PIDController X;
  private PIDController Y;
  private ProfiledPIDController Rotation;
  private HolonomicDriveController controller;

  /* Arm subsystem */
  private ArmSubsystem arm;
  private double armTarget;
  private boolean pistonState;

  /* Claw subsystem */
  private ClawSubsystem claw;
  private boolean grabState;
  private Set<Subsystem> reqs;

  /* Timer stuff */
  // Timer runs relative time from the start of the command
  private Timer timer = new Timer();
  private double timeout;
  private double minTime;

  public AutonomousStep(
      ArmSubsystem arm,
      ClawSubsystem claw,
      DriveSubsystem drive,
      InformationSubsystem info,
      Pose2d target,
      double armState,
      boolean clawGrabState,
      boolean wristPistonState,
      double minTime,
      double timeout) {
    this.arm = arm;
    this.claw = claw;
    this.drive = drive;
    this.info = info;
    this.dest = target;
    this.minTime = minTime;
    this.timeout = timeout;
    armTarget = armState;
    pistonState = wristPistonState;
    grabState = clawGrabState;
    HashSet<Subsystem> set = new HashSet<Subsystem>();
    set.add(drive);
    set.add(arm);
    set.add(claw);
    reqs = set;
  }

  private int loop = 0;

  /** This method should include update calls for all subsystems used in the autonomous program */
  @Override
  public void execute() {
    // According to the source code, timeSeconds, accelerationMetersPerSecond, and
    // curvatureRadPerMeter are ignored, so we can just set those to 0.
    ChassisSpeeds speeds =
        controller.calculate(
            info.getPoseEstimate().toPose2d(), new State(0, 0, 0, dest, 0), dest.getRotation());
    // System.out.println(trajectory.sample(timer.get()).poseMeters);

    if (++loop > 10) {
      System.out.println(
          String.format(
              "%f, %f, %f",
              controller.getXController().getPositionError(),
              controller.getYController().getPositionError(),
              controller.getThetaController().getPositionError()));
    }

    // TODO: with the new kinematics, test if normal control works
    drive.driveFieldRelative(
        speeds.vxMetersPerSecond * Constants.Drive.autonomousDrivetrainCoeff,
        speeds.vyMetersPerSecond * Constants.Drive.autonomousDrivetrainCoeff,
        speeds.omegaRadiansPerSecond * Constants.Drive.autonomousDrivetrainCoeff);

    arm.setpoint(armTarget);
    arm.setClawAlignment(pistonState);

    if (grabState) {
      claw.setTargetState(Value.kForward);
    } else {
      claw.setTargetState(Value.kReverse);
    }
    claw.activate();
  }

  @Override
  public void initialize() {
    X = new PIDController(0.5, 0.08, 0.1);
    Y = new PIDController(0.5, 0.08, 0.1);
    Rotation = new ProfiledPIDController(0.5, 0.1, 0.1, new Constraints(1, 0.5));
    controller = new HolonomicDriveController(X, Y, Rotation);
    // Tolerance of +-5cm & +-1 degrees
    controller.setTolerance(new Pose2d(0.05, 0.05, new Rotation2d(Math.PI / 180)));
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return reqs;
  }

  /**
   * The logic for this should be if all PID loops are at their setpoints, and the minimum time has
   * elapsed, or the time is beyond the timeout, end the step.
   */
  @Override
  public boolean isFinished() {
    return (controller.atReference() && arm.atReference() && timer.get() > minTime)
        || timer.get() > timeout;
  }
}
