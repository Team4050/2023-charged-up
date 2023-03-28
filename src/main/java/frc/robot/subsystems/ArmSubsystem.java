package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  // private final DigitalInput ls1 = new DigitalInput(Constants.Sensors.ArmLimit);

  /* Control */
  // Profiled PID controller. Uses a simple trapezoid contraint as per Dan's request.
  private Constraints constraints = new Constraints(0.2, 0.5);
  // TODO: transition to using built-in PID loop? 1ms resolution, better control.
  private ProfiledPIDController PID = new ProfiledPIDController(0.2, 0.1, 0.1, constraints);
  private double setpoint = pivotMotor.getSelectedSensorPosition();

  /* Misc */
  private final String name = "Arm";

  public ArmSubsystem(ShuffleboardTab tab) {
    // TODO: figure out resolution of integrated gearbox encoder and adjust this value accordingl
    configurePID();

    tab.addDouble(
        "Arm Encoder",
        () -> {
          return pivotMotor.getSelectedSensorPosition();
        });
  }

  @Override
  public void periodic() {
    System.out.println(String.format("%f", pivotMotor.getSelectedSensorPosition(0)));
  }

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
    // TODO: fix the encoder situation and put this on Position control!
    pivotMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  /** Calculates pid output based on the gearbox encoder. */
  public void calculatePID() {
    pivotMotor.set(
        TalonSRXControlMode.Velocity,
        PID.calculate(pivotMotor.getSelectedSensorPosition(), setpoint));
  }

  /**
   * Sets the PID loop setpoint.
   *
   * @param encodedSetpoint The setpoint which the PID loop will try and reach. As of yet there are
   *     no safety limits!
   */
  // TODO: Add setpoint safety limits to this method using ArmLimit A & B in Constants
  public void setpoint(double encodedSetpoint) {
    setpoint = encodedSetpoint;
  }

  /*
   *
   */
  public void configurePID() {
    pivotMotor.configFactoryDefault();
    pivotMotor.configClosedloopRamp(0.1);
    pivotMotor.config_kP(0, 0.2);
    pivotMotor.config_kI(0, 0.1);
    pivotMotor.config_kD(0, 0.1);
    // configure feedforward each time a new setpoint is called for
    pivotMotor.configAllowableClosedloopError(0, 64);
    pivotMotor.configClosedLoopPeriod(0, 1);
    pivotMotor.configClosedLoopPeakOutput(0, 0.4);
  }
}
