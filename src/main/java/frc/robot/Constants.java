// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class Constants {

  public static final class Operator {
    public static final int XboxPrimary = 0;
    public static final int XboxSecondary = 1;
    public static final float DeadzoneMin = 0.05f;
  }

  public static final class Geometry {
    public static final Transform3d RobotToCamera =
        new Transform3d(new Pose3d(), new Pose3d(1f, 0, 0.2f, new Rotation3d()));
  }

  public static final class Drive {
    public static final int FrontLeft = 2;
    public static final int FrontRight = 3;
    public static final int RearLeft = 4;
    public static final int RearRight = 5;
  }

  public static final class Pneumatics {
    public static final int PCM = 1;
    public static final int ClawFwdChannel = 0;
    public static final int ClawRevChannel = 1;
  }
}
