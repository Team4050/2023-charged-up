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
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private ADIS16470_IMU imu = new ADIS16470_IMU(IMUAxis.kZ, Port.kOnboardCS0, CalibrationTime._2s);
  private double Xdev = 0;
  private double Ydev = 0;
  private double Zdev = 0;
  private double[] stdDev = {0, 0, 0, 0, 0, 0};
  private double[] stateDev = {0, 0, 0, 0, 0, 0};
  private double Xmean = 0;
  private double Ymean = 0;
  private double Zmean = 0;
  private double[] mean = {0, 0, 0, 0, 0, 0};
  private int N;

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
    drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                drivetrain.drive(
                    primaryControl.getLeftY(),
                    -primaryControl.getLeftX(),
                    -primaryControl.getRightX()),
            drivetrain));
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
    N++;
    double[] imuData = getImuDataArray();
    for (int i = 0; i < mean.length; i++) {
      mean[i] = ((mean[i] * N) + imuData[i]) / N + 1;
    }

    for (int i = 0; i < stdDev.length; i++) {
      stdDev[i] = ((stdDev[i] * N) + Math.pow((mean[i] - imuData[i]), 2)) / N + 1;
    }

    System.out.println(
        String.format(
            "IMU std devs; accel: %f, %f, %f gyro: %f, %f, %f",
            Math.sqrt(stdDev[0]),
            Math.sqrt(stdDev[1]),
            Math.sqrt(stdDev[2]),
            Math.sqrt(stdDev[3]),
            Math.sqrt(stdDev[4]),
            Math.sqrt(stdDev[5])));
  }

  private double[] getImuDataArray() {
    double[] r = {
      imu.getAccelX(),
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
