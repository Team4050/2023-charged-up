// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.PowerDistribution;
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
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

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

  /*
   **************************************************************************************************
   * Camera & Sensors
   **************************************************************************************************
   */
  private ADIS16470_IMU imu = new ADIS16470_IMU();

  @Log(name = "PowerDistribution")
  private PowerDistribution pdp = new PowerDistribution();

  /*
   **************************************************************************************************
   * Subsystems
   **************************************************************************************************
   */
  private InformationSubsystem info = new InformationSubsystem(imu);
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
    arm.resetEncoder();
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

  public void periodic() {
    Logger.updateEntries();
  }
}
