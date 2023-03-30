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
  private Trigger clawOnTrigger;
  private Trigger clawOffTrigger;
  private Trigger clawWristLeftTrigger;
  private Trigger clawWristRightTrigger;
  public boolean toggle = false;
  private ClawSubsystem claw;
  private Set<Subsystem> requirements;
  private int estimatedBias = 0;

  /**
   * @param clawOnTrigger The controller button to close the claw.
   * @param clawOffTrigger The controller button to open the claw.
   * @param controller The controller that belongs to the secondary driver.
   * @param claw The claw subsystem to use.
   */
  public ClawToggleCmd(
      Trigger clawOnTrigger,
      Trigger clawOffTrigger,
      Trigger clawWristLeftTrigger,
      Trigger clawWristRightTrigger,
      HazardXbox controller,
      ClawSubsystem claw) {
    this.clawOnTrigger = clawOnTrigger;
    this.clawOffTrigger = clawOffTrigger;
    this.clawWristLeftTrigger = clawWristLeftTrigger;
    this.clawWristRightTrigger = clawWristRightTrigger;
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
    clawOnTrigger.onTrue(
        new InstantCommand(
            () -> {
              toggle = true;
            }));

    clawOffTrigger.onTrue(
        new InstantCommand(
            () -> {
              toggle = false;
            }));

    clawWristLeftTrigger.onTrue(
        new InstantCommand(
            () -> {
              claw.setWrist(1);
            }));

    clawWristRightTrigger.onTrue(
        new InstantCommand(
            () -> {
              claw.setWrist(-1);
            }));
  }

  @Override
  public void execute() {
    claw.setWrist(Math.round((float) controller.getLeftX()));

    if (toggle) {
      claw.setTargetState(Value.kForward);
      claw.activate();
      return;
    }
    claw.setTargetState(Value.kReverse);
    claw.activate();
  }
}
