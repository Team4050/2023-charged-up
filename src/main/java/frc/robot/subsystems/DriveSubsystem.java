package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonFX FL = new WPI_TalonFX(Constants.Drive.FrontLeft);
  private final WPI_TalonFX RL = new WPI_TalonFX(Constants.Drive.RearLeft);
  private final WPI_TalonFX FR = new WPI_TalonFX(Constants.Drive.FrontRight);
  private final WPI_TalonFX RR = new WPI_TalonFX(Constants.Drive.RearRight);

  public final Orchestra orchestra = new Orchestra();

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

    orchestra.addInstrument(FL);
    orchestra.addInstrument(RL);
    orchestra.addInstrument(FR);
    orchestra.addInstrument(RR);
  }

  /**
   * Directly sets the drivetrain motor's speeds
   *
   * @param xSpeed The target speed in the horizontal direction
   * @param ySpeed The target speed in the forward/backward direction
   * @param rotation The target rotation velocity (positive is CW)
   */
  public void drive(double xSpeed, double ySpeed, double rotation) {
    if (useImu) {
      System.out.println(imuRef.getAngle());
      drive.driveCartesian(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(imuRef.getAngle()));
      return;
    }
    drive.driveCartesian(xSpeed, ySpeed, rotation);
  }

  /** Use this method to limit the drivetrain's max speed in any direction */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double[] getIMUData() {
    double[] r = {imuRef.getAccelX(), imuRef.getRate(), imuRef.getAccelZ()};
    return r;
  }
}
