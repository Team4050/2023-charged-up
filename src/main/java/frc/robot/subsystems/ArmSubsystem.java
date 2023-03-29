package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
    // configurePID();

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
    if (++loop > 10) {
      loop = 0;
      System.out.println(
          String.format(
              "setpoint: %f, current point: %f",
              setpoint, pivotMotor.getSelectedSensorPosition(0)));
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
    pivotMotor.set(TalonSRXControlMode.Position, setpoint + (speed * 200));
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

    // Sensor must be PulseWidthEncodedPosition
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 1, 10);

    pivotMotor.setSensorPhase(true); //

    pivotMotor.setSelectedSensorPosition(0, 0, 10);
    pivotMotor.setSelectedSensorPosition(0, 1, 10);

    pivotMotor.configClosedloopRamp(0.5);
    pivotMotor.config_kP(0, 0.2);
    pivotMotor.config_kI(0, 0);
    pivotMotor.config_kD(0, 0);
    // configure feedforward each time a new setpoint is called for
    pivotMotor.configAllowableClosedloopError(0, 64);
    pivotMotor.configClosedLoopPeriod(0, 1);
    pivotMotor.configClosedLoopPeakOutput(0, 0.4);
  }
}
