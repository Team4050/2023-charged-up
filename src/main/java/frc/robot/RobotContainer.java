// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Pneumatics;
import frc.robot.commands.ClawToggleCmd;
import frc.robot.commands.DanceCommand;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import io.github.oblarg.oblog.annotations.Log;
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

  private Trigger clawTrigger = secondaryControl.b();
  private Trigger danceTrigger = primaryControl.start();

  /*
   **************************************************************************************************
   * Logging
   **************************************************************************************************
   */
  private DataLog logFile = new DataLog("", LocalDateTime.now().toString() + " log");

  /*
   **************************************************************************************************
   * Camera & Sensors
   **************************************************************************************************
   */
  @Log.CameraStream() private PhotonCamera camera = new PhotonCamera("photonvision");

  @Log.ThreeAxisAccelerometer(name = "ADIS16470 IMU")
  private ADIS16470_IMU imu = new ADIS16470_IMU();

  @Log.PowerDistribution() private PowerDistribution pdp = new PowerDistribution();

  /*
   **************************************************************************************************
   * Subsystems
   **************************************************************************************************
   */
  private DriveSubsystem drivetrain = new DriveSubsystem(imu, logFile);
  private ClawSubsystem claw =
      new ClawSubsystem(
          Pneumatics.PCM,
          PneumaticsModuleType.CTREPCM,
          Pneumatics.ClawFwdChannel,
          Pneumatics.ClawRevChannel);

  /*
   **************************************************************************************************
   * Commands
   **************************************************************************************************
   */
  private ClawToggleCmd clawCmd = new ClawToggleCmd(clawTrigger, claw);
  private DanceCommand dance = new DanceCommand(drivetrain);

  /*
   **************************************************************************************************
   * Container Functions
   **************************************************************************************************
   */
  public RobotContainer() {
    configureBindings();

    // Set the default drive command using input from the primary controller
    drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                drivetrain.driveFieldRelativeSmart(
                    primaryControl.getLeftY(),
                    -primaryControl.getLeftX(),
                    -primaryControl.getRightX()),
            drivetrain));

    claw.setDefaultCommand(clawCmd);
  }

  private void configureBindings() {
    danceTrigger.toggleOnTrue(dance);
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void periodic() {
    drivetrain.logEntries();
    SmartDashboard.putData(pdp);
    SmartDashboard.putData("ADIS IMU", imu);
  }
}
