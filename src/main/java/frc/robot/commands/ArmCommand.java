package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ArmSubsystem;
import java.util.HashSet;
import java.util.Set;

public class ArmCommand extends CommandBase {
  private HazardXbox controller;
  private Trigger clawUpTrigger;
  private Trigger clawDownTrigger;
  private Trigger rest;
  private Trigger score1;
  private Trigger score2;
  private Trigger pickup;
  private Trigger armResetEncoder;
  private ArmSubsystem arm;
  private Set<Subsystem> requirements;

  public ArmCommand(
      ArmSubsystem arm,
      HazardXbox controller,
      Trigger clawUpTrigger,
      Trigger clawDownTrigger,
      Trigger armRest,
      Trigger armScore1,
      Trigger armScore2,
      Trigger armGrab,
      Trigger armResetEncoder) {
    this.controller = controller;
    this.arm = arm;
    this.clawUpTrigger = clawUpTrigger;
    this.clawDownTrigger = clawDownTrigger;
    rest = armRest;
    score1 = armScore1;
    score2 = armScore2;
    pickup = armGrab;
    this.armResetEncoder = armResetEncoder;
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
    rest.onTrue(
        new InstantCommand(
            () -> {
              arm.setpoint(Constants.Operator.ArmRestPosition);
            }));
    score1.onTrue(
        new InstantCommand(
            () -> {
              arm.setpoint(Constants.Operator.ArmLevelOnePosition);
            }));
    score2.onTrue(
        new InstantCommand(
            () -> {
              arm.setpoint(Constants.Operator.ArmLevelTwoPosition);
            }));
    pickup.onTrue(
        new InstantCommand(
            () -> {
              arm.setpoint(Constants.Operator.ArmGrabPosition);
            }));
    armResetEncoder.onTrue(
        new InstantCommand(
            () -> {
              arm.resetEncoder();
            }));
  }

  @Override
  public void execute() {
    arm.setpointAdditive(controller.getRightY() * Constants.Operator.ArmJoystickCoefficient);
    /*if (arm.getArmMotorVelocity() < 48 && arm.getArmError() > 128) {
      controller.rumble(0.4f);
    }*/
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }
}
