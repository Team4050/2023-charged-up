package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ArmSubsystem;
import java.util.HashSet;
import java.util.Set;

public class ArmCommand extends CommandBase {
  private HazardXbox controller;
  private ArmSubsystem arm;
  private Set<Subsystem> requirements;

  public ArmCommand(ArmSubsystem arm, HazardXbox controller) {
    this.controller = controller;
    this.arm = arm;
    requirements = new HashSet<>();
    requirements.add(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.set(controller.getRightY());

    /*if (Math.abs(controller.getRightY()) > 0.1) {
      arm.set(controller.getRightY());
    }*/
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }
}
