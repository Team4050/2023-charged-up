package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ClawSubsystem;
import java.util.HashSet;
import java.util.Set;

public class ClawToggleCmd extends CommandBase {
  private HazardXbox controller;
  private Trigger clawTrigger;
  private Trigger clawHoldTrigger;
  public boolean toggle = false;
  private ClawSubsystem claw;
  private Set<Subsystem> requirements;
  private int estimatedBias = 0;

  public ClawToggleCmd(
      Trigger clawTrigger,
      Trigger clawHoldTrigger,
      Trigger clawFlipTrigger,
      HazardXbox controller,
      ClawSubsystem claw) {
    this.clawTrigger = clawTrigger;
    this.clawHoldTrigger = clawHoldTrigger;
    this.controller = controller;
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

    clawHoldTrigger.whileTrue(
        new InstantCommand(
            () -> {
              toggle = true;
            }));

    clawHoldTrigger.onFalse(
        new InstantCommand(
            () -> {
              toggle = false;
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

    int wristSpeed = (int) controller.getLeftX();
    estimatedBias += wristSpeed;
    System.out.println(estimatedBias);
    // if (Math.abs(estimatedBias) > 30) System.out.println("AAAAAAAAAAAAAAA");
    claw.setWrist(wristSpeed);
  }

  /** Toggles the claw state, link this to a trigger such as a controller button */
  public void toggle() {
    toggle = !toggle;
  }
}
