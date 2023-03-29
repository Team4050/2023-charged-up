package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ArmSubsystem;
import java.util.HashSet;
import java.util.Set;

public class ArmCommand extends CommandBase {
  private HazardXbox controller;
  private Trigger clawUpTrigger;
  private Trigger clawDownTrigger;
  private ArmSubsystem arm;
  private Set<Subsystem> requirements;

  public ArmCommand(
      ArmSubsystem arm, HazardXbox controller, Trigger clawUpTrigger, Trigger clawDownTrigger) {
    this.controller = controller;
    this.arm = arm;
    this.clawUpTrigger = clawUpTrigger;
    this.clawDownTrigger = clawDownTrigger;
    requirements = new HashSet<>();
    requirements.add(arm);
  }

  @Override
  public void initialize() {
    clawUpTrigger.onTrue(
        new InstantCommand(
            () -> {
              arm.setClawAlignment(true);
            }));
    clawDownTrigger.onTrue(
        new InstantCommand(
            () -> {
              arm.setClawAlignment(false);
            }));
  }

  @Override
  public void execute() {
    arm.set(controller.getRightY());
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }
}
