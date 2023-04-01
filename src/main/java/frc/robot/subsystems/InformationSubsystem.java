package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.control.FilteredDrivetrainControl;
import frc.robot.hazard.HazardVision;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class InformationSubsystem extends SubsystemBase {
  /* Sensors */
  private ADIS16470_IMU imu;

  private Timer timer;
  private HazardVision camera = new HazardVision();

  /* Filters & Estimators */
  private MecanumDrivePoseEstimator drivetrainPoseEstimator;
  private Pose3d estimatedPose;
  private FilteredDrivetrainControl filter;
  private Field2d dashboardField;
  private AprilTagFieldLayout layout;

  public InformationSubsystem(ADIS16470_IMU imu, Pose2d startingPose) {
    this.imu = imu;
    // tab.add("ADIS IMU", imu);
    this.camera = new HazardVision();
    // tab.add("Limelight", camera);
    layout = null;

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

    filter = new FilteredDrivetrainControl();
    filter.initialize();

    estimatedPose = new Pose3d(startingPose);

    Optional<EstimatedRobotPose> p = camera.getEstimatedGlobalPose(startingPose);
    if (p.isPresent()) {
      estimatedPose = p.get().estimatedPose;
    }

    timer.reset();
    timer.start();

    dashboardField = new Field2d();
    dashboardField.setRobotPose(startingPose);

    MecanumDriveKinematics kinematics =
        new MecanumDriveKinematics(
            new Translation2d(1, 1),
            new Translation2d(-1, 1),
            new Translation2d(1, -1),
            new Translation2d(-1, -1));
    drivetrainPoseEstimator =
        new MecanumDrivePoseEstimator(
            kinematics,
            new Rotation2d(imu.getAngle()),
            new MecanumDriveWheelPositions(0, 0, 0, 0),
            startingPose);
  }

  public void updatePoseEstimate(double dT, double[] encoderPositions) {
    double[][] columnVec = {{imu.getAccelX() + 0.4}, {imu.getAccelY() + 0.49}, {imu.getRate()}};
    filter.execute(dT, columnVec);

    Pose3d newPose = new Pose3d();
    Optional<EstimatedRobotPose> p = camera.getEstimatedGlobalPose();

    /* TODO: pose estimator does not take imu angle into account.
     * Possibly write custom estimator to take imu data?
     * Removes ambiguity problem
     */

    if (p.isPresent()) {
      dashboardField.setRobotPose(newPose.getX(), newPose.getY(), new Rotation2d());
      // estimatedPose = newPose;
      SmartDashboard.putData("Field", dashboardField);

      if (camera.isTrustworthy())
        drivetrainPoseEstimator.addVisionMeasurement(newPose.toPose2d(), dT);
    }

    drivetrainPoseEstimator.update(
        new Rotation2d(imu.getAngle()),
        new MecanumDriveWheelPositions(
            encoderPositions[0] * Drive.encoderTicksToMeters,
            encoderPositions[1] * Drive.encoderTicksToMeters,
            encoderPositions[2] * Drive.encoderTicksToMeters,
            encoderPositions[3] * Drive.encoderTicksToMeters));

    /*
     * Way to compare photonvision poses with imu data?
     * difference between photonvision positions measured and compared with integrated accel (velocity) data,
     * Kalman filter with vision & accelerometer?
     * xyz pos, xyz vel, avg distance to tags
     *
     * Drivetrain pose estimator implements these
     */
  }

  public Pose3d getPoseEstimate() {
    return estimatedPose;
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPose = camera.getEstimatedGlobalPose(new Pose2d());

    if (estimatedPose.isPresent()) {
      dashboardField.setRobotPose(estimatedPose.get().estimatedPose.toPose2d());
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
