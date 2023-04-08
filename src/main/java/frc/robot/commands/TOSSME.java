package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import java.util.HashSet;
import java.util.Set;

public class TOSSME extends CommandBase {
  private final double grabTiming = 0.09; // 90ms after wrist flick
  private final double end = 0.2; // 200ms timeout
  private Timer timer;
  private ClawSubsystem claw;
  private ArmSubsystem arm;
  private Set<Subsystem> reqs;

  public TOSSME(ArmSubsystem arm, ClawSubsystem claw) {
    this.arm = arm;
    this.claw = claw;
    reqs = new HashSet<Subsystem>();
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
    if (timer.get() > grabTiming) {
      claw.setTargetState(Value.kReverse);
    } else {
      claw.setTargetState(Value.kForward);
    }
    claw.activate();
    arm.setClawAlignment(true);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > end;
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return reqs;
  }
}
