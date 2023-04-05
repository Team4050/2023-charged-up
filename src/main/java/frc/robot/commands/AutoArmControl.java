package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class AutoArmControl extends CommandBase {
  private ArmSubsystem arm;
  private ClawSubsystem claw;
  private double armTarget;
  private boolean pistonState;
  private boolean grabState;

  public AutoArmControl(
      ArmSubsystem arm,
      ClawSubsystem claw,
      double armState,
      boolean clawGrabState,
      boolean wristPistonState) {
    this.arm = arm;
    this.claw = claw;
    armTarget = armState;
    pistonState = wristPistonState;
    grabState = clawGrabState;
  }

  @Override
  public void execute() {
    arm.setpoint(armTarget);
    if (grabState) {
      claw.setTargetState(Value.kForward);
    }
    claw.setTargetState(Value.kReverse);
  }
}
