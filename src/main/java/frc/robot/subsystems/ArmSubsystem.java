package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  // TODO: Piston variable name should be more descriptive when we know what it does
  private final DoubleSolenoid piston =
      new DoubleSolenoid(
          Constants.Pneumatics.PCM,
          Constants.Pneumatics.Module,
          Constants.Pneumatics.ArmFwdChannel,
          Constants.Pneumatics.ArmRevChannel);

  // TODO: Motor variable name should be more descriptive when we know what it does
  private final VictorSP motor = new VictorSP(Constants.Actuators.Arm);

  private final DigitalInput ls1 = new DigitalInput(Constants.Sensors.ArmLimit);

  public ArmSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void test() {
    motor.set(0.3);
  }
}
