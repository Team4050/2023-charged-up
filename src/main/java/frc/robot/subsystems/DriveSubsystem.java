package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
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
  private final MecanumDrive drive = new MecanumDrive(FL, RL, FR, RR);

  private ADIS16470_IMU imu;

  public DriveSubsystem(ADIS16470_IMU imu) {
    FR.setInverted(true);
    RR.setInverted(true);
    this.imu = imu;

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
    drive.driveCartesian(xSpeed, ySpeed, rotation);
  }

  /** Use this method to limit the drivetrain's max speed in any direction */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }
}
