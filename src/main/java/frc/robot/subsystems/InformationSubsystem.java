package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
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
  private Matrix<N3, N1> setpointControlVector;

  private FilteredDrivetrainControl filter;

  public InformationSubsystem(
      ADIS16470_IMU imu,
      Encoder FL,
      Encoder FR,
      Encoder RL,
      Encoder RR,
      PhotonCamera camera,
      Pose2d startingPose) {
    this.imu = imu;
    encoders = new Encoder[] {FL, FR, RL, RR};
    this.camera = camera;
    AprilTagFieldLayout layout = null;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      e.printStackTrace();
    }
    poseEstimator =
        new PhotonPoseEstimator(
            layout, PoseStrategy.LOWEST_AMBIGUITY, camera, Geometry.RobotToCamera);

    filter = new FilteredDrivetrainControl(imu);
    filter.initialize();

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
    double[][] columnVec = {{imu.getAccelX() + 0.29}, {imu.getAccelY() + 0.29}, {imu.getAccelZ()}};
    filter.execute(dT, columnVec);
    setpointControlVector = filter.getControlVector();

    estimatedPose.set(
        0,
        0,
        estimatedPose.get(0, 0) + filter.getStateEstimate().get(0, 0) * (Math.pow(dT, 2) / 2));

    estimatedPose.set(
        0,
        1,
        estimatedPose.get(0, 1) + filter.getStateEstimate().get(0, 1) * (Math.pow(dT, 2) / 2));

    estimatedPose.set(
        0,
        2,
        estimatedPose.get(0, 2) + filter.getStateEstimate().get(0, 2) * (Math.pow(dT, 2) / 2));

    SmartDashboard.putNumberArray(
        "Filtered position estimate",
        new double[] {estimatedPose.get(0, 0), estimatedPose.get(1, 0), estimatedPose.get(2, 0)});
  }
}
