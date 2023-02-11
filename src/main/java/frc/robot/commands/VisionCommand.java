package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import org.photonvision.*;
import org.photonvision.targeting.*;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionCommand extends CommandBase {
  private static PhotonCamera camera;
  private static DriveSubsystem drivetrain;
  private static CommandXboxController joystick;

  private static PIDController turnController;

  public VisionCommand(PhotonCamera c, DriveSubsystem d, CommandXboxController j) {
    camera = c;
    drivetrain = d;
    joystick = j;

    addRequirements(d);
  }

  @Override
  public void initialize() {
    super.initialize();
    turnController = new PIDController(1, 0, 1);
  }

  @Override
  public void execute() {
    PhotonPipelineResult latestImage = camera.getLatestResult();

    if (latestImage.hasTargets()) {
      // get the closest target (list is sorted by tag area)
      PhotonTrackedTarget target = latestImage.targets.get(0);

      // PID controller to align with target's rotation
      drivetrain.setRotation(-turnController.calculate(target.getYaw(), 0));
    }
  }

  @Override
  public void cancel() {
    drivetrain.drive(0, 0, 0);
    drivetrain.go();
  }
}
