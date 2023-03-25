package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /* Pistons & Motors */
  private final DoubleSolenoid clawAlignmentPiston =
      new DoubleSolenoid(
          Constants.Pneumatics.PCM,
          Constants.Pneumatics.Module,
          Constants.Pneumatics.ArmFwdChannel,
          Constants.Pneumatics.ArmRevChannel);

  private final TalonSRX pivotMotor = new TalonSRX(Constants.Actuators.Arm);
  // The motor encoder that's supposedly built into the gearbox. Uses MXP port channels.

  /* Sensors */
  private final Encoder pivotGearboxEncoder = new Encoder(0, 0);
  private final DigitalInput ls1 = new DigitalInput(Constants.Sensors.ArmLimit);

  /* Control */
  // Profiled PID controller. Uses a simple trapezoid contraint as per Dan's request.
  private Constraints constraints = new Constraints(0, 0);
  private ProfiledPIDController PID = new ProfiledPIDController(0.1, 0, 0, constraints);
  private double setpoint = 0;

  /* Misc */
  private final String name = "Arm";

  public ArmSubsystem() {
    // TODO: figure out resolution of integrated gearbox encoder and adjust this value accordingly
    pivotGearboxEncoder.setDistancePerPulse(1.0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public String getName() {
    return name;
  }

  /**
   * Directly sets the desired arm speed. Use for manual arm control?
   *
   * @param speed The motor speed.
   */
  public void set(double speed) {
    pivotMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  /** Calculates pid output based on the gearbox encoder. */
  public void calculatePID() {
    pivotMotor.set(
        TalonSRXControlMode.Velocity, PID.calculate(pivotGearboxEncoder.get(), setpoint));
  }

  /**
   * Sets the PID loop setpoint.
   *
   * @param encodedSetpoint The setpoint which the PID loop will try and reach. As of yet there are
   *     no safety limits!
   */
  public void setpoint(double encodedSetpoint) {
    setpoint = encodedSetpoint;
  }
}
