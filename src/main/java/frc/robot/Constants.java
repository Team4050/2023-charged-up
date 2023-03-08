// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {
  public static final class Operator {
    public static final int XboxPrimary = 0;
    public static final int XboxSecondary = 1;
    public static final float DeadzoneMin = 0.05f;
  }

  public static final class Drive {
    public static final int FrontLeft = 2;
    public static final int FrontRight = 3;
    public static final int RearLeft = 4;
    public static final int RearRight = 5;
  }

  public static final class Actuators {
    public static final int Arm = 6;
  }

  public static final class Sensors {
    public static final int ClawLimitL = 0;
    public static final int CLawLimitR = 1;
    public static final int ArmLimit = 2;
  }

  public static final class Pneumatics {
    public static final int PCM = 1;
    public static final PneumaticsModuleType Module = PneumaticsModuleType.CTREPCM;
    public static final int ClawFwdChannel = 0;
    public static final int ClawRevChannel = 1;
    public static final int ArmFwdChannel = 2;
    public static final int ArmRevChannel = 3;
  }
}
