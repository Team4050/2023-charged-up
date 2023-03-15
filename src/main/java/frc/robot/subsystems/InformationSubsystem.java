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
  private Matrix<N3, N1> setpointControlVector;

  private FilteredDrivetrainControl filter;
  private Field2d dashboardField;

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
    this.camera = new PhotonCamera("photonvision");
    camera = this.camera;
    AprilTagFieldLayout layout = null;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      e.printStackTrace();
    }
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

    // System.out.println(String.format("%f %f %f", newPose.getX(), newPose.getY(),
    // newPose.getZ()));
  }
}
