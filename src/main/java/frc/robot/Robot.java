// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    DriverStation.reportWarning(
        String.join(
            "",
            "",
            "______ _       _                            _     ___ _____ _____  _____\n",
            "| ___ (_)     | |                          | |   /   |  _  |  ___||  _  |\n",
            "| |_/ /_  ___ | |__   __ _ ______ _ _ __ __| |  / /| | |/' |___ \\ | |/' |\n",
            "| ___ \\ |/ _ \\| '_ \\ / _` |_  / _` | '__/ _` | / /_| |  /| |   \\ \\|  /| |\n",
            "| |_/ / | (_) | | | | (_| |/ / (_| | | | (_| | \\___  \\ |_/ /\\__/ /\\ |_/ /\n",
            "\\____/|_|\\___/|_| |_|\\__,_/___\\__,_|_|  \\__,_|     |_/\\___/\\____/  \\___/\n"),
        false);
    DriverStation.reportWarning(
        String.join(
            "",
            "",
            "              ____   ____  _\n",
            "             |_  _| |_  _|(_)\n ",
            "               \\ \\   / /  __  _ .--.   .---.  _ .--.\n ",
            "                \\ \\ / /  [  |[ '/'`\\ \\/ /__\\\\[ `/'`\\]\n",
            "                  \\ ' /    | | | \\__/ || \\__., | |\n",
            "                   \\_/    [___]| ;.__/  '.__.'[___]\n",
            "                               [__|\n"),
        false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(robotContainer.getAutonomousCommand());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
