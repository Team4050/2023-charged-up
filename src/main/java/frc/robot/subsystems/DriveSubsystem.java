package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
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
  private final IntegerArrayLogEntry encoderLogger;
  private final FloatArrayLogEntry imuLogger;

  @Log.MecanumDrive(name = "Drive")
  private final MecanumDrive drive = new MecanumDrive(FL, RL, FR, RR);

  public final Orchestra orchestra = new Orchestra();
  private ADIS16470_IMU imu;

  private final String name = "Drivetrain";

  public DriveSubsystem(ADIS16470_IMU imu, DataLog log) {
    FR.setInverted(true);
    RR.setInverted(true);
    this.imu = imu;

    orchestra.addInstrument(FL);
    orchestra.addInstrument(RL);
    orchestra.addInstrument(FR);
    orchestra.addInstrument(RR);

    encoderLogger = new IntegerArrayLogEntry(log, "/robot/" + name);
    imuLogger = new FloatArrayLogEntry(log, "/robot/ADIS16470_IMU");
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

  public void driveFieldRelative(double xSpeed, double ySpeed, double rotation) {
    drive.driveCartesian(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(imu.getAngle()));
  }

  /** Use this method to limit the drivetrain's max speed in any direction */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  @Override
  public String getName() {
    return name;
  }

  public void logEntries() {
    encoderLogger.append(new long[] {0, 0, 0, 0});
    imuLogger.append(
        new float[] {(float) imu.getAccelX(), (float) imu.getAccelY(), (float) imu.getAccelZ()});
  }
}
