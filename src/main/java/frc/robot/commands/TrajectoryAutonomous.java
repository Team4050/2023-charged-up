package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import java.util.ArrayList;

public class TrajectoryAutonomous extends CommandBase {
  private DriveSubsystem drive;
  private InformationSubsystem info;
  private ArrayList<Pose2d> trajectory1;
  private DriveToPosition subcommand;

  public TrajectoryAutonomous() {}

  @Override
  public void initialize() {
    trajectory1 = new ArrayList<Pose2d>(0);
    Pose2d startingPose = info.getPoseEstimate().toPose2d();
    ArrayList<Pose2d> list = new ArrayList<Pose2d>();
    list.add(startingPose);
    list.add(
        new Pose2d(startingPose.getX() + 1, startingPose.getY() + 1, startingPose.getRotation()));
    subcommand = new DriveToPosition(drive, info, list);
  }

  @Override
  public void execute() {
    subcommand.execute();
  }
}
