package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ClawSubsystem extends SubsystemBase implements Loggable {
  @Log(name = "Claw Piston")
  private DoubleSolenoid piston;

  @Log(name = "Wrist Motor")
  private WPI_TalonSRX wristMotor = new WPI_TalonSRX(Constants.Actuators.Wrist);

  private Value target;
  private double wristLimit = 0.75;

  public ClawSubsystem() {
    piston =
        new DoubleSolenoid(
            Constants.Pneumatics.PCM,
            Constants.Pneumatics.Module,
            Constants.Pneumatics.ClawFwdChannel,
            Constants.Pneumatics.ClawRevChannel);
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

  public void setWrist(double speed) {
    wristMotor.set(ControlMode.PercentOutput, speed * 0.75); // wristLimit
  }

  @Config(name = "Wrist Max Output", defaultValueNumeric = 75)
  public void setWristLimit(int limit) {
    wristLimit = limit / 100;
  }
}
