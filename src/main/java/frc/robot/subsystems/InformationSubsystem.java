package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hazard.HazardVision;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class InformationSubsystem extends SubsystemBase implements Loggable {

  private Timer timer;
  private HazardVision camera = new HazardVision();

  @Log(name = "IMU")
  private ADIS16470_IMU imu;

  @Log(name = "Robot Position")
  private Field2d field = new Field2d();

  public InformationSubsystem(ADIS16470_IMU imu) {
    this.imu = imu;

    timer = new Timer();
    timer.start();
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPose = camera.getEstimatedGlobalPose(null);

    if (estimatedPose.isPresent()) {
      field.setRobotPose(estimatedPose.get().estimatedPose.toPose2d());
    }
  }

  public enum axis {
    XAcc,
    YAcc,
    ZAcc,
    ZRot,
    ZRate
  }

  /**
   * Pretty self-explainatory. Gets raw data from the IMU.
   *
   * @param a The type of data to return.
   * @return The requested data.
   */
  public double getData(axis a) {
    switch (a) {
      case XAcc:
        return imu.getAccelX();
      case YAcc:
        return imu.getAccelY();
      case ZAcc:
        return imu.getAccelZ();
      case ZRate:
        return imu.getRate();
      case ZRot:
        return imu.getAngle();
      default:
        return 0;
    }
  }

  /**
   * Returns multiple forms of raw IMU data in one call.
   *
   * @param a An array containing axis. The returned array with have corresponding values.
   * @return An array of IMU data. The type of data is requested through the a parameter.
   */
  public double[] getData(axis[] a) {
    double[] r = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      r[i] = getData(a[i]);
    }
    return r;
  }

  public enum motor {
    FL(0),
    FR(1),
    RL(2),
    RR(3);

    private int value;

    private motor(int v) {
      this.value = v;
    }
  }

  /**
   * For autonomous: drives in the supplied direction for the supplied amount of time
   *
   * @param drive The drivetrain subsystem to use.
   * @param direction The direction to drive in.
   * @param timeout If the command takes longer than the supplied value, the command stops.
   */
  public void driveUntil(DriveSubsystem drive, Pose2d direction, int timeout) {
    double since = timer.get();
    while (timer.get() < since + timeout) {
      drive.driveSmart(direction.getX(), direction.getY(), 0);
    }
  }
}
