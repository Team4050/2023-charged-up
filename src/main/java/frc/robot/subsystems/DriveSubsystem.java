package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase implements Loggable {
  @Log.MotorController(name = "FL")
  private final WPI_TalonSRX FL = new WPI_TalonSRX(Constants.Drive.FrontLeft);

  @Log.MotorController(name = "RL")
  private final WPI_TalonSRX RL = new WPI_TalonSRX(Constants.Drive.RearLeft);

  @Log.MotorController(name = "FR")
  private final WPI_TalonSRX FR = new WPI_TalonSRX(Constants.Drive.FrontRight);

  @Log.MotorController(name = "RR")
  private final WPI_TalonSRX RR = new WPI_TalonSRX(Constants.Drive.RearRight);

  @Log.MecanumDrive(name = "Drive")
  private final MecanumDrive drive = new MecanumDrive(FL, RL, FR, RR);

  private ADIS16470_IMU imu;

  public DriveSubsystem(ADIS16470_IMU imu) {
    FR.setInverted(true);
    RR.setInverted(true);
    this.imu = imu;
  }

  /**
   * Directly sets the drivetrain motor's speeds
   *
   * @param xSpeed The target speed in the horizontal direction
   * @param ySpeed The target speed in the forward/backward direction
   * @param rotation The target rotation velocity (positive is CW)
   */
  public void drive(double xSpeed, double ySpeed, double rotation) {
    drive.driveCartesian(xSpeed, ySpeed, rotation);
  }

  /** Use this method to limit the drivetrain's max speed in any direction */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }
}
