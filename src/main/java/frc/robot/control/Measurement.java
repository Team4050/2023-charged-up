package frc.robot.control;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class Measurement {
  private int FLEncoder;
  private int FREncoder;
  private int RLEncoder;
  private int RREncoder;

  private double XAccel;
  private double YAccel;

  private double YawAccel;

  public Measurement(int FLE, int FRE, int RLE, int RRE, ADIS16448_IMU IMU) {}
}
