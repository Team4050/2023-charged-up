package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonFX FL = new WPI_TalonFX(Constants.Drive.FrontLeft);
  private final WPI_TalonFX RL = new WPI_TalonFX(Constants.Drive.RearLeft);
  private final WPI_TalonFX FR = new WPI_TalonFX(Constants.Drive.FrontRight);
  private final WPI_TalonFX RR = new WPI_TalonFX(Constants.Drive.RearRight);

  public final Orchestra orchestra = new Orchestra();

  private final MecanumDrive drive = new MecanumDrive(FL, RL, FR, RR);

  public DriveSubsystem() {
    FR.setInverted(true);
    RR.setInverted(true);

    orchestra.addInstrument(FL);
    orchestra.addInstrument(RL);
    orchestra.addInstrument(FR);
    orchestra.addInstrument(RR);
  }

  public void drive(double xSpeed, double ySpeed, double rotation) {
    drive.driveCartesian(xSpeed, ySpeed, rotation);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }
}
