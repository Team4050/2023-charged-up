// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ClawToggleCmd;
import frc.robot.commands.DanceCommand;
import frc.robot.commands.TOSSME;
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
  private Trigger tossTrigger = secondaryControl.povUp();
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

  private SendableChooser<String> autonomousSwitch = new SendableChooser<>();
  private final String noCmd = "no";
  private final String simpleShort = "short";
  private final String simpleCmd = "simple";
  private final String simpleLong = "long";

  /*
   **************************************************************************************************
   * Camera & Sensors
   **************************************************************************************************
   */
  private ADIS16470_IMU imu = new ADIS16470_IMU();

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
    autonomousSwitch.addOption("Exit community 2 seconds", simpleShort);
    autonomousSwitch.addOption("Exit community 2.7 seconds", simpleCmd);
    autonomousSwitch.addOption("Exit community 3 seconds", simpleLong);
    Shuffleboard.getTab("Autonomous").add(autonomousSwitch);

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
    tossTrigger.onTrue(new TOSSME(arm, claw));
  }

  /**
   * Gets the selected autonomous command
   *
   * @return A command which controls various robot subsystems and accomplishes autonomous tasks.
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new AutonomousCommand(
            drivetrain,
            arm,
            claw,
            info,
            new Pose2d(0, 0, new Rotation2d()),
            2,
            true,
            Constants.Operator.ArmLevelTwoPosition),
        new AutonomousCommand(
            drivetrain,
            arm,
            claw,
            info,
            new Pose2d(0, 0, new Rotation2d()),
            2,
            false,
            Constants.Operator.ArmLevelTwoPosition),
        new AutonomousCommand(
            drivetrain, arm, claw, info, new Pose2d(0.3, 0, new Rotation2d()), 3.2, false, 0));
    /*switch (autonomousSwitch.getSelected()) {
      case noCmd:
        return new AutonomousCommand(drivetrain, arm, claw, info, 2);
      case simpleShort:
        return new AutonomousCommand(drivetrain, arm, claw, info, 2);
      case simpleCmd:
        return new AutonomousCommand(drivetrain, arm, claw, info, 2.7);
      case simpleLong:
        return new AutonomousCommand(drivetrain, arm, claw, info, 3);
      default:
        return new AutonomousCommand(drivetrain, arm, claw, info, 2.7);
    }*/
  }

  public void periodic() {
    Logger.updateEntries();
  }
}
