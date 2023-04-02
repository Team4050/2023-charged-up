package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.InformationSubsystem.axis;

public class DriveSubsystem extends SubsystemBase {

  /* Motor Controllers */
  private WPI_TalonFX FL = new WPI_TalonFX(Constants.Drive.FrontLeft);
  private WPI_TalonFX RL = new WPI_TalonFX(Constants.Drive.RearLeft);
  private WPI_TalonFX FR = new WPI_TalonFX(Constants.Drive.FrontRight);
  private WPI_TalonFX RR = new WPI_TalonFX(Constants.Drive.RearRight);

  private MecanumDrive drive = new MecanumDrive(FL, RL, FR, RR);

  private boolean autocorrection = false;

  /* Misc */
  public Orchestra orchestra = new Orchestra();
  private InformationSubsystem info;
  private PIDController spinController = new PIDController(0.125, 0.1, 0);

  public DriveSubsystem(InformationSubsystem info) {
    this.info = info;

    // Set up drive motors
    FR.setInverted(true);
    RR.setInverted(true);

    // Set up orchestra
    orchestra.addInstrument(FL);
    orchestra.addInstrument(RL);
    orchestra.addInstrument(FR);
    orchestra.addInstrument(RR);
  }

  /**
   * Directly sets the drivetrain motor's speeds.
   *
   * @param xSpeed The target speed in the horizontal direction
   * @param ySpeed The target speed in the forward/backward direction
   * @param rotation The target rotation velocity (positive is CW)
   */
  public void drive(double xSpeed, double ySpeed, double rotation) {
    drive.driveCartesian(xSpeed, ySpeed, rotation * Operator.RotationDamping);
  }

  /**
   * Directly sets the drivetrain motor's speeds relative to the field.
   *
   * @param xSpeed The target speed in the horizontal direction
   * @param ySpeed The target speed in the forward/backward direction
   * @param rotation The target rotation velocity (positive is CW)
   */
  public void driveFieldRelative(double xSpeed, double ySpeed, double rotation) {
    drive.driveCartesian(
        xSpeed,
        ySpeed,
        rotation * Operator.RotationDamping,
        Rotation2d.fromDegrees(info.getData(axis.ZRate)));
  }

  /**
   * Drives in robot-relative coordinates with rotation damping (if damping is enabled on the
   * dashboard.)
   *
   * @param xSpeed X velocity
   * @param ySpeed Y velocity
   * @param rotation Rotational velocity
   */
  public void driveSmart(double xSpeed, double ySpeed, double rotation) {
    double v = spinController.calculate(info.getData(axis.ZRate), 0);
    if (rotation == 0 && autocorrection) {
      rotation = v / 50;
    }
    drive.driveCartesian(xSpeed, ySpeed, rotation * Operator.RotationDamping);
  }

  /**
   * Drives in field-relative coordinates with rotation damping (if damping is enabled on the
   * dashboard.)
   *
   * @param xSpeed X velocity in field coordinates
   * @param ySpeed Y velocity in field coordinates
   * @param rotation Rotational velocity
   */
  public void driveFieldRelativeSmart(double xSpeed, double ySpeed, double rotation) {
    double v = spinController.calculate(info.getData(axis.ZRate), 0);
    if (rotation == 0 && autocorrection) {
      rotation = v / 50;
    }
    drive.driveCartesian(
        xSpeed,
        ySpeed,
        rotation * Operator.RotationDamping,
        Rotation2d.fromDegrees(info.getData(axis.ZRot)));
  }

  /**
   * Limit drivetrain max output
   *
   * @param maxOutput Maximum output value (0-1)
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput / 100);
  }

  /**
   * Gets a specified integrated motor encoder value
   *
   * @param motor The motor number (0-4 for drivetrain)
   * @return
   */
  public double getSensorValue(int motor) {
    switch (motor) {
      case 0:
        return FL.getSelectedSensorPosition();
      case 1:
        return FR.getSelectedSensorPosition();
      case 2:
        return RL.getSelectedSensorPosition();
      case 3:
        return RR.getSelectedSensorPosition();
      default:
        return 0;
    }
  }

  public void setAutocorrection(boolean enabled) {
    autocorrection = enabled;
  }
}
