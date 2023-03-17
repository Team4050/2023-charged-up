package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Geometry;
import frc.robot.control.FilteredDrivetrainControl;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class InformationSubsystem extends SubsystemBase {
  private ADIS16470_IMU imu;
  private Encoder[] encoders;
  private PhotonCamera camera;

  private PhotonPoseEstimator poseEstimator;
  private Matrix<N3, N1> estimatedPose;
  private Matrix<N3, N1> setpoint;
  private Matrix<N3, N1> setpointControlVector;

  private FilteredDrivetrainControl filter;
  private Field2d dashboardField;

  private PIDController X;
  private PIDController Y;
  private PIDController Z;

  public InformationSubsystem(
      ADIS16470_IMU imu,
      Encoder FL,
      Encoder FR,
      Encoder RL,
      Encoder RR,
      Encoder Arm,
      PhotonCamera camera,
      Pose2d startingPose) {
    this.imu = imu;
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

    filter = new FilteredDrivetrainControl(imu);
    filter.initialize();

    dashboardField = new Field2d();
    dashboardField.setRobotPose(startingPose);

    estimatedPose =
        new Matrix<N3, N1>(
            new SimpleMatrix(
                new double[][] {
                  {startingPose.getX()},
                  {startingPose.getY()},
                  {startingPose.getRotation().getDegrees()}
                }));
  }

  public void updatePoseEstimate(double dT) {
    double[][] columnVec = {{imu.getAccelX() + 0.4}, {imu.getAccelY() + 0.49}, {imu.getRate()}};
    filter.execute(dT, columnVec);
    setpointControlVector = filter.getControlVector();

    estimatedPose.set(0, 0, estimatedPose.get(0, 0) + (filter.getStateEstimate().get(0, 0) * dT));

    estimatedPose.set(1, 0, estimatedPose.get(1, 0) + (filter.getStateEstimate().get(1, 0) * dT));

    estimatedPose.set(2, 0, estimatedPose.get(2, 0) + (filter.getStateEstimate().get(2, 0) * dT));

    Pose3d newPose = new Pose3d();
    if (poseEstimator.update().isPresent()) {
      newPose = poseEstimator.update().get().estimatedPose;
      dashboardField.setRobotPose(newPose.getX(), newPose.getY(), new Rotation2d());
      SmartDashboard.putData("Field", dashboardField);
    }
  }

  public Matrix<N3, N1> getPoseEstimate() {
    return estimatedPose;
  }

  public void ResetPID() {
    X.reset();
    Y.reset();
    Z.reset();
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
}
