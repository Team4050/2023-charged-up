package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawSubsystem;
import java.util.HashSet;
import java.util.Set;

public class ClawToggleCmd extends CommandBase {
  private Trigger clawTrigger;
  public boolean toggle = false;
  private ClawSubsystem claw;
  private Set<Subsystem> requirements;

  public ClawToggleCmd(Trigger clawTrigger, ClawSubsystem claw) {
    this.clawTrigger = clawTrigger;
    this.claw = claw;
    requirements = new HashSet<Subsystem>();
    requirements.add(claw);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }

  @Override
  public void initialize() {
    clawTrigger.onTrue(
        new InstantCommand(
            () -> {
              toggle();
            }));
  }

  @Override
  public void execute() {
    if (toggle) {
      claw.setTargetState(Value.kForward);
      claw.activate();
      return;
    }
    claw.setTargetState(Value.kReverse);
    claw.activate();
  }

  /** Toggles the claw state, link this to a trigger such as a controller button */
  public void toggle() {
    toggle = !toggle;
  }
}
