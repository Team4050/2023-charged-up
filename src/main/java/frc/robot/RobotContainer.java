// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ClawToggleCmd;
import frc.robot.commands.DanceCommand;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import java.time.LocalDateTime;
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

  /*
   * Discuss controls with drive team
   *
   *
   *
   */
  private Trigger clawOpenTrigger = secondaryControl.leftTrigger();
  private Trigger clawClosedTrigger = secondaryControl.rightTrigger();
  private Trigger clawUpTrigger = null;
  private Trigger clawDownTrigger = null;
  private Trigger clawWristLeftTrigger = secondaryControl.leftBumper();
  private Trigger clawWristRightTrigger = secondaryControl.rightBumper();
  private Trigger armPosGrabTrigger = secondaryControl.a();
  private Trigger armPosScore1Trigger = secondaryControl.x();
  private Trigger armPosScore2Trigger = secondaryControl.y();
  private Trigger armPosRestTrigger = secondaryControl.b();
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
  private PhotonCamera camera = new PhotonCamera("photonvision");
  private ADIS16470_IMU imu = new ADIS16470_IMU();
  private PowerDistribution pdp = new PowerDistribution();

  /*
   **************************************************************************************************
   * Subsystems
   **************************************************************************************************
   */
  private InformationSubsystem info = new InformationSubsystem(dashboardTab, imu, camera, null);
  private DriveSubsystem drivetrain = new DriveSubsystem(info, logFile, dashboardTab);
  private ClawSubsystem claw = new ClawSubsystem(dashboardTab);
  private ArmSubsystem arm = new ArmSubsystem(dashboardTab);

  /*
   **************************************************************************************************
   * Commands
   **************************************************************************************************
   */
  private ClawToggleCmd clawCmd =
      new ClawToggleCmd(clawOpenTrigger, clawClosedTrigger, secondaryControl, claw);
  private ArmCommand armCmd = new ArmCommand(arm, secondaryControl, clawUpTrigger, clawDownTrigger);
  private DanceCommand dance = new DanceCommand(drivetrain);

  /*
   **************************************************************************************************
   * Container Functions
   **************************************************************************************************
   */
  public RobotContainer() {
    Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    compressor.disable();
    System.out.println(compressor.getPressureSwitchValue());
    compressor.close();

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

    arm.setDefaultCommand(armCmd);
    claw.setDefaultCommand(clawCmd);

    CameraServer.startAutomaticCapture();
    // CameraServer.getInstance().startAutomaticCapture();
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

  /** Called every 20ms(?) */
  public void periodic() {
    // SmartDashboard.putData(pdp);
  }
}
