package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Geometry;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class InformationSubsystem extends SubsystemBase {
  private ADIS16470_IMU imu;
  private Encoder[] encoders;
  private PhotonCamera camera;

  private PhotonPoseEstimator poseEstimator;
  private Pose2d estimatedPose;

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
  }
}
