package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Actuators;
import frc.robot.Constants.Pneumatics;

public class ClawSubsystem extends SubsystemBase {
  private DoubleSolenoid piston;
  private Value target;
  private final TalonSRX wristMotor = new TalonSRX(Actuators.Wrist);
  private static final double wristLimit = 0.75;

  public ClawSubsystem(ShuffleboardTab tab) {
    piston =
        new DoubleSolenoid(
            Pneumatics.PCM,
            PneumaticsModuleType.CTREPCM,
            Pneumatics.ClawFwdChannel,
            Pneumatics.ClawRevChannel);

    tab.add("Claw grab (Currently wrist control)", piston);
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

  public void setWrist(int speed) {
    wristMotor.set(ControlMode.PercentOutput, speed * wristLimit);
  }
}
