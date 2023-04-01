package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ArmSubsystem extends SubsystemBase implements Loggable {
  /* Pistons & Motors */
  @Log(name = "Wrist Piston")
  private DoubleSolenoid clawAlignmentPiston =
      new DoubleSolenoid(
          Constants.Pneumatics.PCM,
          Constants.Pneumatics.Module,
          Constants.Pneumatics.ArmFwdChannel,
          Constants.Pneumatics.ArmRevChannel);

  @Log(name = "Arm Position")
  private WPI_TalonSRX pivotMotor = new WPI_TalonSRX(Constants.Actuators.Arm);
  // The motor encoder that's supposedly built into the gearbox. Uses MXP port channels.

  /* Sensors */
  // private final DigitalInput ls1 = new DigitalInput(Constants.Sensors.ArmLimit);

  /* Control */
  private double home = 0;
  private double setpoint = 0;

  public ArmSubsystem() {
    configurePID();
    setpoint = 0;
    pivotMotor.set(ControlMode.Position, 0);
    // home = pivotMotor.getSelectedSensorPosition();
  }

  private int loop = 0;

  @Override
  public void periodic() {
    // pivotMotor.set(TalonSRXControlMode.Position, setpoint + 100);
    softLimit(pivotMotor.getSelectedSensorPosition(0));

    // TODO: become confident enough to remove logging
    loop++;
    if (loop > 10) {
      loop = 0;
      System.out.println(
          String.format(
              "setpoint: %f, current point: %f",
              home + setpoint, pivotMotor.getSelectedSensorPosition(0)));
    }
  }

  /**
   * Directly sets the desired arm speed. Use for manual arm control?
   *
   * @param speed The motor speed.
   */
  @Deprecated
  public void set(double speed) {
    setpoint = speed;
    pivotMotor.set(TalonSRXControlMode.PercentOutput, home + setpoint);
    // pivotMotor.set(TalonSRXControlMode.PercentOutput, speed);

    // System.out.println(pivotMotor.getSelectedSensorPosition());
  }

  /**
   * Sets the PID loop setpoint.
   *
   * @param encodedSetpoint The setpoint which the PID loop will try and reach.
   */
  public void setpoint(double encodedSetpoint) {
    setpoint = encodedSetpoint;
    setpoint = limit(setpoint);
    pivotMotor.set(TalonSRXControlMode.Position, home + setpoint);
  }

  /**
   * Sets the PID loop setpoint.
   *
   * @param add The amount to add to the current setpoint.
   */
  public void setpointAdditive(double add) {
    setpoint += add;
    setpoint = limit(setpoint);
    pivotMotor.set(TalonSRXControlMode.Position, home + setpoint);
  }

  /** Resets the arm encoder. Currently disabled. */
  public void resetEncoder() {
    // pivotMotor.setSelectedSensorPosition(0);
    // home = pivotMotor.getSelectedSensorPosition();
  }

  /**
   * Sets the PID loop setpoint to the current position of the arm. No guarantee it'll stop
   * immediately though.
   */
  public void PIDhaltArm() {
    pivotMotor.set(TalonSRXControlMode.Position, pivotMotor.getSelectedSensorPosition());
  }

  public double getArmMotorVelocity() {
    return pivotMotor.getSelectedSensorVelocity();
  }

  public double getArmError() {
    return pivotMotor.getClosedLoopError(0);
  }

  public void setClawAlignment(boolean up) {
    if (up) {
      clawAlignmentPiston.set(Value.kForward);
      return;
    }
    clawAlignmentPiston.set(Value.kReverse);
  }

  public void softLimit(double v) {
    if (v < Constants.Operator.ArmEncoderLimitLow || v > Constants.Operator.ArmEncoderLimitHigh) {
      pivotMotor.set(TalonSRXControlMode.PercentOutput, 0);
      pivotMotor.setNeutralMode(NeutralMode.Brake);
      pivotMotor.disable();
    }
  }

  public double limit(double v) {
    if (v < Constants.Operator.ArmEncoderLimitLow) v = Constants.Operator.ArmEncoderLimitLow;
    if (v > Constants.Operator.ArmEncoderLimitHigh) v = Constants.Operator.ArmEncoderLimitHigh;
    return v;
  }

  public void configurePID() {
    pivotMotor.configFactoryDefault();

    // Sensor must be PulseWidthEncodedPosition
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 1, 10);

    pivotMotor.setInverted(
        InvertType.None); // TODO: decide if inverting the arm control makes more sense
    pivotMotor.setSensorPhase(true);

    // pivotMotor.setSelectedSensorPosition(0, 0, 10);
    // pivotMotor.setSelectedSensorPosition(0, 1, 10);

    pivotMotor.configClosedloopRamp(0.5);
    pivotMotor.config_kP(0, 0.65);
    pivotMotor.config_kI(0, 0.1);
    pivotMotor.configMaxIntegralAccumulator(0, 1);
    pivotMotor.config_kD(0, 0.1);
    // configure feedforward each time a new setpoint is called for
    pivotMotor.configAllowableClosedloopError(0, 24);
    pivotMotor.configClosedLoopPeriod(0, 4);
    pivotMotor.configClosedLoopPeakOutput(0, 1);

    pivotMotor.setNeutralMode(NeutralMode.Coast);
  }
}
