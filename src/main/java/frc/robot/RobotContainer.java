// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonomousStep;
import frc.robot.commands.ClawToggleCmd;
import frc.robot.commands.DanceCommand;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import io.github.oblarg.oblog.Logger;

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

  // Claw grabbing control
  private Trigger clawOpenTrigger = secondaryControl.leftTrigger();
  private Trigger clawClosedTrigger = secondaryControl.rightTrigger();
  // Claw alingment piston
  // TODO: currently set to the controller dpad, discuss with drive team
  private Trigger clawUpTrigger = secondaryControl.leftBumper();
  private Trigger clawDownTrigger = secondaryControl.rightBumper();
  // Claw rotation motor
  // private Trigger clawWristLeftTrigger = secondaryControl.leftBumper();
  // private Trigger clawWristRightTrigger = secondaryControl.rightBumper();
  // Arm setpoint buttons
  private Trigger armPosGrabTrigger = secondaryControl.a();
  private Trigger armPosScore1Trigger = secondaryControl.x();
  private Trigger armPosScore2Trigger = secondaryControl.y();
  private Trigger armPosRestTrigger = secondaryControl.b();
  // Resets the arm encoder
  private Trigger armResetTrigger = secondaryControl.back();
  // :(
  private Trigger danceTrigger = primaryControl.start();

  // TODO: figure out controller rumble?

  private SendableChooser<String> autonomousSwitch = new SendableChooser<>();
  private final String noCmd = "no";
  private final String simpleCmd = "simple";
  private final String danceCmd = "crab";
  private final String trajectory = "traj";

  /*
   **************************************************************************************************
   * Camera & Sensors
   **************************************************************************************************
   */
  // @Log.ThreeAxisAccelerometer(name = "ADIS16470 IMU")
  public ADIS16470_IMU imu = new ADIS16470_IMU(IMUAxis.kZ, Port.kOnboardCS0, CalibrationTime._4s);

  private double[] stdDev = {0, 0, 0, 0, 0, 0};
  private double[] stateDev = {0, 0, 0, 0, 0, 0};
  private double[] mean = {0, 0, 0, 0, 0, 0};
  private double[] fixedMean = {-0.362191, -0.287574, 9.784017, -0.083874, -2.076228, -1.730506};
  private double[] previousState = {0, 0, 0, 0, 0, 0};
  private int N = 0;

  /*
   **************************************************************************************************
   * Subsystems
   **************************************************************************************************
   */
  private InformationSubsystem info = new InformationSubsystem(imu, new Pose2d());
  private DriveSubsystem drivetrain = new DriveSubsystem(info);
  private ClawSubsystem claw = new ClawSubsystem();
  private ArmSubsystem arm = new ArmSubsystem();

  /*
   **************************************************************************************************
   * Commands
   **************************************************************************************************
   */
  private ClawToggleCmd clawCmd =
      new ClawToggleCmd(clawOpenTrigger, clawClosedTrigger, secondaryControl, claw);
  private ArmCommand armCmd =
      new ArmCommand(
          arm,
          secondaryControl,
          clawUpTrigger,
          clawDownTrigger,
          armPosRestTrigger,
          armPosScore1Trigger,
          armPosScore2Trigger,
          armPosGrabTrigger,
          armResetTrigger);
  private DanceCommand dance = new DanceCommand(drivetrain);

  /*
   **************************************************************************************************
   * Container Functions
   **************************************************************************************************
   */
  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);

    autonomousSwitch.setDefaultOption("No auto", noCmd);
    autonomousSwitch.addOption("Exit community", simpleCmd);
    autonomousSwitch.addOption("Dance", danceCmd);
    autonomousSwitch.addOption("Trajectory Test", trajectory);

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

    arm.resetEncoder();
    arm.setDefaultCommand(armCmd);
    arm.resetEncoder();
    System.out.println(arm.getEncoderValue());
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
  public SequentialCommandGroup getAutonomousCommand() {
    // return new AutonomousCommand(drivetrain, arm, claw, info);
    /*
    DriveToPosition command1 =
        new DriveToPosition(
            drivetrain,
            info,
            new Pose2d(
                info.getPoseEstimate().toPose2d().getX() - 40,
                info.getPoseEstimate().toPose2d().getY() - 40,
                info.getPoseEstimate().toPose2d().getRotation()));
    DriveToPosition command2 =
        new DriveToPosition(
            drivetrain,
            info,
            new Pose2d(
                info.getPoseEstimate().toPose2d().getX(),
                info.getPoseEstimate().toPose2d().getY(),
                info.getPoseEstimate().toPose2d().getRotation()));
    DriveToPosition command3 =
        new DriveToPosition(
            drivetrain,
            info,
            new Pose2d(
                info.getPoseEstimate().toPose2d().getX() + 40,
                info.getPoseEstimate().toPose2d().getY() + 40,
                info.getPoseEstimate().toPose2d().getRotation())); */

    /* New autonomous is based on steps.
     * Steps' min time should not be greater than the timeout.
     * Timeout should not be greater than 15 seconds.
     * Arm state should be within the soft limits for the arm.
     *
     */

    // TODO: code a real autonomous mode based on field measurements
    Pose2d relativeStart = info.getPoseEstimate().toPose2d();
    AutonomousStep step0 =
        new AutonomousStep(
            arm,
            claw,
            drivetrain,
            info,
            new Pose2d(
                relativeStart.getX() + 20, relativeStart.getY(), relativeStart.getRotation()),
            0,
            true,
            false,
            0,
            10);
    AutonomousStep step1 =
        new AutonomousStep(
            arm,
            claw,
            drivetrain,
            info,
            new Pose2d(
                relativeStart.getX() + 20, relativeStart.getY(), relativeStart.getRotation()),
            Constants.Operator.ArmLevelTwoPosition,
            true,
            false,
            0,
            15);
    AutonomousStep step2 =
        new AutonomousStep(
            arm,
            claw,
            drivetrain,
            info,
            new Pose2d(
                relativeStart.getX() + 20, relativeStart.getY(), relativeStart.getRotation()),
            Constants.Operator.ArmLevelTwoPosition,
            false,
            false,
            1,
            15);
    AutonomousStep step3 =
        new AutonomousStep(
            arm,
            claw,
            drivetrain,
            info,
            new Pose2d(
                relativeStart.getX() - 60, relativeStart.getY(), relativeStart.getRotation()),
            0,
            false,
            false,
            0,
            10);

    // I hope this is how you use command groups
    SequentialCommandGroup cmdGroup = new SequentialCommandGroup(step0, step1, step2, step3);

    return cmdGroup;
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

  private int loop = 0;
  /** Called every 20ms(?) */
  public void periodic(double dT) {
    info.updatePoseEstimate(dT, drivetrain.getWheelPositions());

    if (++loop > 10) {
      System.out.println(
          String.format(
              "%f, %f, %f",
              info.getPoseEstimate().getX(),
              info.getPoseEstimate().getY(),
              info.getPoseEstimate().getRotation().getAngle()));
      loop = 0;
    }

    Logger.updateEntries();
  }
}
