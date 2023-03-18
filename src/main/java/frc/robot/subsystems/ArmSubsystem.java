package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final DoubleSolenoid clawAlignmentPiston =
      new DoubleSolenoid(
          Constants.Pneumatics.PCM,
          Constants.Pneumatics.Module,
          Constants.Pneumatics.ArmFwdChannel,
          Constants.Pneumatics.ArmRevChannel);

  private final TalonSRX pivotMotor = new TalonSRX(Constants.Actuators.Arm);

  private final DigitalInput ls1 = new DigitalInput(Constants.Sensors.ArmLimit);

  private PIDController PIDhold = new PIDController(0.1, 0, 0);

  public ArmSubsystem() {}

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  /** Test method, please ignore */
  public void test() {
    pivotMotor.set(TalonSRXControlMode.Velocity, 0);
  }

  public void set(double speed) {
    pivotMotor.set(TalonSRXControlMode.Velocity, speed);
  }
}
