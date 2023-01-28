package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonSRX FL = new WPI_TalonSRX(Constants.Drive.FrontLeft);
  private final WPI_TalonSRX RL = new WPI_TalonSRX(Constants.Drive.RearLeft);
  private final WPI_TalonSRX FR = new WPI_TalonSRX(Constants.Drive.FrontRight);
  private final WPI_TalonSRX RR = new WPI_TalonSRX(Constants.Drive.RearRight);

  private final MecanumDrive drive = new MecanumDrive(FL, RL, FR, RR);

  private double xSpeed;
  private double ySpeed;
  private double rotation;

  public DriveSubsystem() {
    FR.setInverted(true);
    RR.setInverted(true);
  }

  public void drive(double xSpeed, double ySpeed, double rotation) {
    setXSpeed(xSpeed);
    setYSpeed(ySpeed);
    setRotation(rotation);
  }

  public void setXSpeed(double xSpeed) {
    this.xSpeed = xSpeed;
  }

  public void setYSpeed(double ySpeed) {
    this.ySpeed = ySpeed;
  }

  public void setRotation(double rotation) {
    this.rotation = rotation;
  }

  public void go() {
    this.drive.driveCartesian(this.xSpeed, this.ySpeed, this.rotation);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
