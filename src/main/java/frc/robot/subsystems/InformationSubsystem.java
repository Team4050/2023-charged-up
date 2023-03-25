package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Geometry;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class InformationSubsystem extends SubsystemBase {
  /* Sensors */
  private ADIS16470_IMU imu;
  private Encoder[] encoders;
  private PhotonCamera camera;
  private Timer timer;

  /* Filters & Estinators */
  private PhotonPoseEstimator poseEstimator;
  private Pose2d estimatedPose;

  public InformationSubsystem(
      ShuffleboardTab tab,
      ADIS16470_IMU imu,
      Encoder FL,
      Encoder FR,
      Encoder RL,
      Encoder RR,
      Encoder Arm,
      PhotonCamera camera,
      Pose2d startingPose) {
    this.imu = imu;
    tab.add("ADIS IMU", imu);
    // this.camera = camera;
    // tab.add("Limelight", camera);
    encoders = new Encoder[] {FL, FR, RL, RR, Arm};
    AprilTagFieldLayout layout = null;

    // Setup field layout from resource file
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      e.printStackTrace();
    }

    /*
     * Setup pose estimator to average from low ambiguity targets, aka measure from the targets that are the most descernable
     *
     * Tends to give semi-acurrate data but with very fast fluxuation.
     * Control loops using this should rely more on their I component.
     */
    poseEstimator =
        new PhotonPoseEstimator(
            layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, Geometry.RobotToCamera);

    timer = new Timer();
    timer.start();
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
    RR(3),
    ArmPivot(4);

    private int value;

    private motor(int v) {
      this.value = v;
    }
  }

  /**
   * Gets readngs from the motor encoders. Since the encoders still aren't on the robot, their
   * accuracy is unknown.
   *
   * @param motor
   * @return
   */
  public int getReading(motor motor) {
    return encoders[motor.value].get();
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
