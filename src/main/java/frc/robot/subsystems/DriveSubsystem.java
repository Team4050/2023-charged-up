package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase implements Loggable{
  @log.MotorController(name = "FL")
  private final WPI_TalonSRX FL = new WPI_TalonSRX(Constants.Drive.FrontLeft);
  @log.MotorController(name = "RL")
  private final WPI_TalonSRX RL = new WPI_TalonSRX(Constants.Drive.RearLeft);
  @log.MotorController(name = "FR")
  private final WPI_TalonSRX FR = new WPI_TalonSRX(Constants.Drive.FrontRight);
  @log.MotorController(namw = "RR")
  private final WPI_TalonSRX RR = new WPI_TalonSRX(Constants.Drive.RearRight);

  @log.MecanumDrive(name = "Drive")
  private final MecanumDrive drive = new MecanumDrive(FL, RL, FR, RR);

  private double xSpeed;
  private double ySpeed;
  private double rotation;

  public DriveSubsystem() {
    FR.setInverted(true);
    RR.setInverted(true);
  }

  public void drive(double xSpeed, double ySpeed, double rotation) {
    this.drive.driveCartesian(this.xSpeed, this.ySpeed, this.rotation);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }
}
