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

  public void SetTargetState(Value state) {
    target = state;
  }

  public void Activate() {
    piston.set(target);
  }
}
