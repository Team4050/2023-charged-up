package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public final MecanumDrive drive = new MecanumDrive(FL, RL, FR, RR);

  @Log.Accelerometer(name = "ADIS16470 IMU")
  @Log.Gyro(name = "ADIS16470 IMU")
  public static ADIS16470_IMU imuRef;

  private boolean useImu = true;

  public DriveSubsystem(ADIS16470_IMU imuReference) {
    imuRef = imuReference;
    FR.setInverted(true);
    RR.setInverted(true);
  }

  public void drive(double xSpeed, double ySpeed, double rotation) {
    if (useImu) {
      System.out.println(imuRef.getAngle());
      drive.driveCartesian(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(imuRef.getAngle()));
      return;
    }
    drive.driveCartesian(xSpeed, ySpeed, rotation);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double[] getIMUData() {
    double[] r = {
      imuRef.getAccelX(),
      imuRef.getAccelY(),
      imuRef.getAccelZ(),
      Math.cos(imuRef.getAngle()),
      Math.sin(imuRef.getAngle()),
      imuRef.getRate(),
      0,
      0,
      0,
      0
    };
    return r;
  }
}
