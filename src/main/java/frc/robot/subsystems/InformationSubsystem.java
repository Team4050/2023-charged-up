package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Geometry;
import frc.robot.control.FilteredDrivetrainControl;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class InformationSubsystem extends SubsystemBase {
  /* Sensors */
  private ADIS16470_IMU imu;
  private Encoder[] encoders;
  private PhotonCamera camera;
  private Timer timer;

  /* Filters & Estimators */
  private PhotonPoseEstimator poseEstimator;
  private Matrix<N3, N1> estimatedPose;
  private FilteredDrivetrainControl filter;
  private Field2d dashboardField;

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
    this.camera = camera;
    tab.add("Limelight", camera);
    encoders = new Encoder[] {FL, FR, RL, RR, Arm};
    this.camera = new PhotonCamera("photonvision");
    camera = this.camera;
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
            layout, PoseStrategy.LOWEST_AMBIGUITY, this.camera, Geometry.RobotToCamera);

    filter = new FilteredDrivetrainControl();
    filter.initialize();

    estimatedPose =
        new Matrix<N3, N1>(
            new SimpleMatrix(
                new double[][] {
                  {startingPose.getX()},
                  {startingPose.getY()},
                  {startingPose.getRotation().getDegrees()}
                }));

    Optional<EstimatedRobotPose> p = poseEstimator.update();
    if (p.isPresent()) {
      estimatedPose =
          new Matrix<N3, N1>(
              new SimpleMatrix(
                  new double[][] {
                    {p.get().estimatedPose.getX()},
                    {p.get().estimatedPose.getY()},
                    {p.get().estimatedPose.getRotation().getZ()}
                  }));
    }

    timer.reset();
    timer.start();

    dashboardField = new Field2d();
    dashboardField.setRobotPose(startingPose);
  }

  public void updatePoseEstimate(double dT) {
    double[][] columnVec = {{imu.getAccelX() + 0.4}, {imu.getAccelY() + 0.49}, {imu.getRate()}};
    filter.execute(dT, columnVec);

    estimatedPose.set(0, 0, estimatedPose.get(0, 0) + (filter.getStateEstimate().get(0, 0) * dT));

    estimatedPose.set(1, 0, estimatedPose.get(1, 0) + (filter.getStateEstimate().get(1, 0) * dT));

    estimatedPose.set(2, 0, estimatedPose.get(2, 0) + (filter.getStateEstimate().get(2, 0) * dT));

    Pose3d newPose = new Pose3d();
    Optional<EstimatedRobotPose> p = poseEstimator.update();
    /* TODO: pose estimator does not take imu angle into account.
     * Possibly write custom estimator to take imu data?
     * Removes ambiguity problem
     */
    if (p.isPresent()) {
      try {
        newPose = poseEstimator.update().get().estimatedPose;
      } catch (Exception e) {
        e.printStackTrace();
        newPose = new Pose3d();
      }
      dashboardField.setRobotPose(newPose.getX(), newPose.getY(), new Rotation2d());
      estimatedPose =
          new Matrix<N3, N1>(
              new SimpleMatrix(
                  new double[][] {{newPose.getX()}, {newPose.getY()}, {imu.getAngle()}}));
      SmartDashboard.putData("Field", dashboardField);
    }
    /*
     * Way to compare photonvision poses with imu data?
     * difference between photonvision positions measured and compared with integrated accel (velocity) data,
     * Kalman filter with vision & accelerometer?
     * xyz pos, xyz vel, avg distance to tags
     */
  }

  public Matrix<N3, N1> getPoseEstimate() {
    return estimatedPose;
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
