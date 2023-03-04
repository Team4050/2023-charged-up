// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Pneumatics;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* Control Interface */
  HazardXbox primaryControl =
      new HazardXbox(Constants.Operator.XboxPrimary, Constants.Operator.DeadzoneMin);
  HazardXbox secondaryControl =
      new HazardXbox(Constants.Operator.XboxSecondary, Constants.Operator.DeadzoneMin);

  /* Camera & Sensors */
  private PhotonCamera camera = new PhotonCamera("photonvision");
  public ADIS16470_IMU imu = new ADIS16470_IMU(IMUAxis.kZ, Port.kOnboardCS0, CalibrationTime._2s);
  private double[] stdDev = {0, 0, 0, 0, 0, 0};
  private double[] stateDev = {0, 0, 0, 0, 0, 0};
  private double[] mean = {0, 0, 0, 0, 0, 0};
  private double[] fixedMean = {-0.362191, -0.287574, 9.784017, -0.083874, -2.076228, -1.730506};
  private double[] previousState = {0, 0, 0, 0, 0, 0};
  private int N = 0;

  /* Subsystems */
  private DriveSubsystem drivetrain = new DriveSubsystem(imu);
  private ClawSubsystem claw =
      new ClawSubsystem(
          Pneumatics.PCM,
          PneumaticsModuleType.CTREPCM,
          Pneumatics.ClawFwdChannel,
          Pneumatics.ClawRevChannel);

  /* Commands */
  // No commands yet

  public RobotContainer() {
    configureBindings();

    // Set the default drive command using input from the primary controller
    /*drivetrain.setDefaultCommand(
    new RunCommand(
        () ->
            drivetrain.drive(
                primaryControl.getLeftY(),
                -primaryControl.getLeftX(),
                -primaryControl.getRightX()),
        drivetrain));*/
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    primaryControl
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  claw.setTargetState(Value.kForward);
                  claw.activate();
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  claw.setTargetState(Value.kReverse);
                  claw.activate();
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
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
    // System.out.println(
    //  String.format("IMU raw data; %f, %f, %f", imuData[0], imuData[1], imuData[2]));
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
}
