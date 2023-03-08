package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ClawSubsystem;
import java.util.HashSet;
import java.util.Set;

class ClawToggleCmd extends CommandBase {
  private HazardXbox controller;
  public boolean toggle = false;
  private ClawSubsystem claw;
  private Set<Subsystem> requirements;

  public ClawToggleCmd(HazardXbox controller, ClawSubsystem claw) {
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
    controller
        .b()
        .onTrue(
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
