package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  private DoubleSolenoid piston;
  private Value target;

  public ClawSubsystem(
      int module, PneumaticsModuleType moduleType, int fwdChannel, int revChannel) {
    piston = new DoubleSolenoid(module, moduleType, fwdChannel, revChannel);
  }

  /**
   * Sets the target value for the double solenoid
   *
   * @param state The target value (Forward, Reverse, or Off)
   */
  public void setTargetState(Value state) {
    target = state;
  }

  /** Powers the piston, moving it to the target state */
  public void activate() {
    piston.set(target);
  }
}
