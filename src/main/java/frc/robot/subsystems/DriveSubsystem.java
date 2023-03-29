package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.InformationSubsystem.axis;
import java.util.Map;

public class DriveSubsystem extends SubsystemBase {

  /* Motor Controllers */
  private final WPI_TalonFX FL = new WPI_TalonFX(Constants.Drive.FrontLeft);
  private final WPI_TalonFX RL = new WPI_TalonFX(Constants.Drive.RearLeft);
  private final WPI_TalonFX FR = new WPI_TalonFX(Constants.Drive.FrontRight);
  private final WPI_TalonFX RR = new WPI_TalonFX(Constants.Drive.RearRight);

  private final MecanumDrive drive = new MecanumDrive(FL, RL, FR, RR);

  /* Loggers */
  private final String name = "Drivetrain";

  private final SendableChooser<String> autoControlSwitch = new SendableChooser<>();
  private final String off = "Autocorrection disabled";
  private final String on = "Autocorrection enabled";
  private ShuffleboardTab dTab;
  private GenericEntry maxSpeed;

  /* Misc */
  public final Orchestra orchestra = new Orchestra();
  private final InformationSubsystem info;
  private final PIDController spinController = new PIDController(0.125, 0.1, 0);

  public DriveSubsystem(InformationSubsystem info, ShuffleboardTab tab) {
    this.info = info;

    dTab = tab;

    maxSpeed =
        dTab.add("Max speed", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();

    // Set up drive motors
    // This might be what is preventing the orchestra from playing
    FR.setInverted(true);
    RR.setInverted(true);

    // Set up orchestra
    orchestra.addInstrument(FL);
    orchestra.addInstrument(RL);
    orchestra.addInstrument(FR);
    orchestra.addInstrument(RR);

    // Set up loggers
    autoControlSwitch.setDefaultOption("off", off);
    autoControlSwitch.addOption("on", on);
    dTab.add("Autocorrection", autoControlSwitch);
    dTab.add("Mecanum", drive).withWidget(BuiltInWidgets.kMecanumDrive);
  }

  @Override
  public void periodic() {
    drive.setMaxOutput(maxSpeed.getDouble(1.0));
  }

  /**
   * Directly sets the drivetrain motor's speeds.
   *
   * @param xSpeed The target speed in the horizontal direction
   * @param ySpeed The target speed in the forward/backward direction
   * @param rotation The target rotation velocity (positive is CW)
   */
  public void drive(double xSpeed, double ySpeed, double rotation) {
    drive.driveCartesian(xSpeed, ySpeed, rotation * Operator.RotationDamping);
  }

  /**
   * Directly sets the drivetrain motor's speeds relative to the field.
   *
   * @param xSpeed The target speed in the horizontal direction
   * @param ySpeed The target speed in the forward/backward direction
   * @param rotation The target rotation velocity (positive is CW)
   */
  public void driveFieldRelative(double xSpeed, double ySpeed, double rotation) {
    drive.driveCartesian(
        xSpeed,
        ySpeed,
        rotation * Operator.RotationDamping,
        Rotation2d.fromDegrees(info.getData(axis.ZRate)));
  }

  /**
   * Drives in robot-relative coordinates with rotation damping (if damping is enabled on the
   * dashboard.)
   *
   * @param xSpeed X velocity in field coordinates
   * @param ySpeed Y velocity in field coordinates
   * @param rotation Rotational velocity
   */
  public void driveSmart(double xSpeed, double ySpeed, double rotation) {
    double v = spinController.calculate(info.getData(axis.ZRate), 0);
    if (rotation == 0 && autoControlSwitch.getSelected() == on) {
      rotation = v / 50;
    }
    drive.driveCartesian(xSpeed, ySpeed, rotation * Operator.RotationDamping);
  }

  /**
   * Drives in field-relative coordinates with rotation damping (if damping is enabled on the
   * dashboard.)
   *
   * @param xSpeed X velocity in field coordinates
   * @param ySpeed Y velocity in field coordinates
   * @param rotation Rotational velocity
   */
  public void driveFieldRelativeSmart(double xSpeed, double ySpeed, double rotation) {
    double v = spinController.calculate(info.getData(axis.ZRate), 0);
    if (rotation == 0 && autoControlSwitch.getSelected() == on) {
      rotation = v / 50;
    }
    drive.driveCartesian(
        xSpeed,
        ySpeed,
        rotation * Operator.RotationDamping,
        Rotation2d.fromDegrees(info.getData(axis.ZRot)));
  }

  /**
   * Limit drivetrain max output
   *
   * @param maxOutput Maximum output value (0-1)
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /**
   * Gets a specified integrated motor encoder value
   *
   * @param motor The motor number (0-4 for drivetrain)
   * @return
   */
  public double getSensorValue(int motor) {
    switch (motor) {
      case 0:
        return FL.getSelectedSensorPosition();
      case 1:
        return FR.getSelectedSensorPosition();
      case 2:
        return RL.getSelectedSensorPosition();
      case 3:
        return RR.getSelectedSensorPosition();
      default:
        return 0;
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
