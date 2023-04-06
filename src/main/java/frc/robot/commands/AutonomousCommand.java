package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import java.util.HashSet;
import java.util.Set;

public class AutonomousCommand extends CommandBase {
  private DriveSubsystem drivetrain;
  private Pose2d speed;
  private ArmSubsystem arm;
  private ClawSubsystem claw;
  private InformationSubsystem info;
  private double time;
  private boolean grab;
  private double armPos;
  private Timer timer;
  private Set<Subsystem> reqs;

  public AutonomousCommand(
      DriveSubsystem drive,
      ArmSubsystem arm,
      ClawSubsystem claw,
      InformationSubsystem info,
      Pose2d speed,
      double time,
      boolean grab,
      double armPos) {
    this.drivetrain = drive;
    this.arm = arm;
    this.claw = claw;
    this.info = info;
    this.time = time;
    this.speed = speed;
    this.grab = grab;
    this.armPos = armPos;
    reqs = new HashSet<Subsystem>();
    reqs.add(drive);
    reqs.add(arm);
    reqs.add(claw);
  }

  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    drivetrain.drive(speed.getX(), speed.getY(), 0);
    if (grab) {
      claw.setTargetState(Value.kForward);
    } else {
      claw.setTargetState(Value.kReverse);
    }
    claw.activate();

    arm.setpoint(armPos);
  }

  @Override
  public boolean isFinished() {
    return (timer.get() > time);
  }

  public Set<Subsystem> getRequirements() {
    return reqs;
  }
}
