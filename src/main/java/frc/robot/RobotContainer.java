// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Pneumatics;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ClawToggleCmd;
import frc.robot.commands.DanceCommand;
import frc.robot.commands.HoldPosition;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import io.github.oblarg.oblog.annotations.Log;
import java.time.LocalDateTime;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonCamera;

public class RobotContainer {
  /*
   **************************************************************************************************
   * Control Interface
   **************************************************************************************************
   */
  private HazardXbox primaryControl =
      new HazardXbox(Constants.Operator.XboxPrimary, Constants.Operator.DeadzoneMin);
  private HazardXbox secondaryControl =
      new HazardXbox(Constants.Operator.XboxSecondary, Constants.Operator.DeadzoneMin);

  private Trigger clawTrigger = secondaryControl.b();
  private Trigger danceTrigger = primaryControl.start();

  private SendableChooser<String> autonomousSwitch = new SendableChooser<>();
  private final String noCmd = "no";
  private final String simpleCmd = "simple";
  private final String danceCmd = "crab";

  /*
   **************************************************************************************************
   * Logging
   **************************************************************************************************
   */
  private DataLog logFile = new DataLog("", LocalDateTime.now().toString() + " log");
  private ShuffleboardTab dashboardTab = Shuffleboard.getTab("Custom");

  /*
   **************************************************************************************************
   * Camera & Sensors
   **************************************************************************************************
   */
  @Log.CameraStream() private PhotonCamera camera = new PhotonCamera("photonvision");

  @Log.ThreeAxisAccelerometer(name = "ADIS16470 IMU")
  public ADIS16470_IMU imu = new ADIS16470_IMU(IMUAxis.kZ, Port.kOnboardCS0, CalibrationTime._4s);

  private double[] stdDev = {0, 0, 0, 0, 0, 0};
  private double[] stateDev = {0, 0, 0, 0, 0, 0};
  private double[] mean = {0, 0, 0, 0, 0, 0};
  private double[] fixedMean = {-0.362191, -0.287574, 9.784017, -0.083874, -2.076228, -1.730506};
  private double[] previousState = {0, 0, 0, 0, 0, 0};
  private int N = 0;

  @Log.PowerDistribution() private PowerDistribution pdp = new PowerDistribution();

  /*
   **************************************************************************************************
   * Subsystems
   **************************************************************************************************
   */
  private InformationSubsystem info =
      new InformationSubsystem(
          dashboardTab,
          imu,
          null,
          null,
          null,
          null,
          null,
          camera,
          new Pose2d(0, 0, new Rotation2d()));
  private DriveSubsystem drivetrain = new DriveSubsystem(info, logFile, dashboardTab);
  private ClawSubsystem claw =
      new ClawSubsystem(
          Pneumatics.PCM,
          PneumaticsModuleType.CTREPCM,
          Pneumatics.ClawFwdChannel,
          Pneumatics.ClawRevChannel);
  private ArmSubsystem arm = new ArmSubsystem();

  /*
   **************************************************************************************************
   * Commands
   **************************************************************************************************
   */
  private ClawToggleCmd clawCmd = new ClawToggleCmd(clawTrigger, claw);
  private ArmCommand armCmd = new ArmCommand(arm, secondaryControl);
  private DanceCommand dance = new DanceCommand(drivetrain);

  /*
   **************************************************************************************************
   * Container Functions
   **************************************************************************************************
   */
  public RobotContainer() {
    autonomousSwitch.setDefaultOption("No auto", noCmd);
    autonomousSwitch.addOption("Exit community", simpleCmd);
    autonomousSwitch.addOption("Dance", danceCmd);

    configureBindings();

    // Set the default drive command using input from the primary controller
    drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                drivetrain.driveSmart(
                    primaryControl.getLeftY(),
                    -primaryControl.getLeftX(),
                    -primaryControl.getRightX()),
            drivetrain));

    drivetrain.setDefaultCommand(
        new HoldPosition(
            drivetrain,
            info,
            new Matrix<N3, N1>(new SimpleMatrix(new double[][] {{12}, {4}, {0}})),
            primaryControl));

    arm.setDefaultCommand(armCmd);
    claw.setDefaultCommand(clawCmd);
  }

  /** Maps commands to their respective triggers. */
  private void configureBindings() {
    danceTrigger.toggleOnTrue(dance);
  }

  /**
   * Gets the selected autonomous command
   *
   * @return A command which controls various robot subsystems and accomplishes autonomous tasks.
   */
  public Command getAutonomousCommand() {
    switch (autonomousSwitch.getSelected()) {
      case noCmd:
        return null;
      case simpleCmd:
        return new AutonomousCommand(drivetrain, arm, claw, info);
      case danceCmd:
        return new DanceCommand(drivetrain);
      default:
        return null;
    }
  }

  public void manualLogging() {
    System.out.println(
        String.format(
            "IMU avgs; %f, %f, %f, %f, %f, %f",
            mean[0], mean[1], mean[2], mean[3], mean[4], mean[5]));

    System.out.println(
        String.format(
            "IMU std devs; accel: %f, %f, %f gyro: %f, %f, %f prediction + measurement noise: %f, %f, %f, %f, %f, %f",
            Math.sqrt(stdDev[0]),
            Math.sqrt(stdDev[1]),
            Math.sqrt(stdDev[2]),
            Math.sqrt(stdDev[3]),
            Math.sqrt(stdDev[4]),
            Math.sqrt(stdDev[5]),
            Math.sqrt(stateDev[0]),
            Math.sqrt(stateDev[1]),
            Math.sqrt(stateDev[2]),
            Math.sqrt(stateDev[3]),
            Math.sqrt(stateDev[4]),
            Math.sqrt(stateDev[5])));
    // cov(v + w) = (cov(v) + cov(w)) / 2?
  }

  public void updateAvg(double dT) {
    double[] imuData = getImuDataArray();
    double[] predictedState = predictState(previousState, dT);

    for (int i = 0; i < mean.length; i++) {
      mean[i] = ((mean[i] * N) + imuData[i]) / (double) (N + 1);
    }

    for (int i = 0; i < stdDev.length; i++) {
      stdDev[i] = ((stdDev[i] * N) + Math.pow((fixedMean[i] - imuData[i]), 2)) / (double) (N + 1);
      stateDev[i] =
          ((stateDev[i] * N) + Math.pow((predictedState[i] - imuData[i]), 2)) / (double) (N + 1);
    }

    N++;
  }

  private double[] getImuDataArray() {
    double[] r = {
      imu.getAccelX(),
      // java.util.random.RandomGenerator.getDefault().nextDouble(),
      imu.getAccelY(),
      imu.getAccelZ(),
      imu.getAngle(),
      imu.getXComplementaryAngle(),
      imu.getYComplementaryAngle(),
      imu.getRate()
    };
    return r;
  }

  private double[] predictState(double[] state, double t) {
    state[3] += imu.getRate() * t;
    return state;
  }

  /** Called every 20ms(?) */
  public void periodic(double dT) {
    info.updatePoseEstimate(dT, drivetrain.getWheelPositions());
    SmartDashboard.putData(pdp);
    SmartDashboard.putData(imu);
  }
}
