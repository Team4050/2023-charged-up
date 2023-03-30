package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

  private final WPI_TalonSRX pivotMotor = new WPI_TalonSRX(Constants.Actuators.Arm);
  // The motor encoder that's supposedly built into the gearbox. Uses MXP port channels.

  /* Sensors */
  // private final DigitalInput ls1 = new DigitalInput(Constants.Sensors.ArmLimit);

  /* Control */
  // Profiled PID controller. Uses a simple trapezoid contraint as per Dan's request.
  private Constraints constraints = new Constraints(0.2, 0.5);
  // TODO: transition to this from using built-in PID loop?
  private ProfiledPIDController PID = new ProfiledPIDController(0.2, 0.1, 0.1, constraints);
  private double home = pivotMotor.getSelectedSensorPosition();
  private double setpoint = 0;

  /* Misc */
  private final String name = "Arm";

  public ArmSubsystem(ShuffleboardTab tab) {
    configurePID();
    home = pivotMotor.getSelectedSensorPosition();

    tab.addDouble(
        "Arm Encoder",
        () -> {
          return pivotMotor.getSelectedSensorPosition();
        });
  }

  private int loop = 0;

  @Override
  public void periodic() {
    // pivotMotor.set(TalonSRXControlMode.Position, setpoint + 100);
    // TODO: remove logging when arm encoder is fixed
    loop++;
    if (loop > 10) {
      loop = 0;
      System.out.println(
          String.format(
              "setpoint: %f, current point: %f",
              home + setpoint, pivotMotor.getSelectedSensorPosition(0)));
    }
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
    setpoint = speed;
    pivotMotor.set(TalonSRXControlMode.Position, home + setpoint);

    if (pivotMotor.getSelectedSensorPosition() < Constants.Operator.ArmEncoderLimitLow
        || pivotMotor.getSelectedSensorPosition() > Constants.Operator.ArmEncoderLimitHigh) {
      pivotMotor.set(TalonSRXControlMode.PercentOutput, 0);
      pivotMotor.disable();
    }
    // pivotMotor.set(TalonSRXControlMode.PercentOutput, speed);

    // System.out.println(pivotMotor.getSelectedSensorPosition());
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

  public void setClawAlignment(boolean up) {
    if (up) {
      clawAlignmentPiston.set(Value.kForward);
      return;
    }
    clawAlignmentPiston.set(Value.kReverse);
  }

  /*
   *
   */
  public void configurePID() {
    pivotMotor.configFactoryDefault();

    // Sensor must be PulseWidthEncodedPosition
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 1, 10);

    pivotMotor.setInverted(
        InvertType.None); // TODO: decide if inverting the arm control makes more sense
    pivotMotor.setSensorPhase(true); // TODO: see if this fixes the reversing issue

    pivotMotor.setSelectedSensorPosition(0, 0, 10);
    // pivotMotor.setSelectedSensorPosition(0, 1, 10);

    pivotMotor.configClosedloopRamp(0.5);
    pivotMotor.config_kP(0, 0.8);
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
