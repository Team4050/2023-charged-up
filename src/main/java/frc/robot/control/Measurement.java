package frc.robot.control;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class Measurement {
  // FL, FR, RL, RR
  public int[] encoders;
  public float[] powers;

  public double XAccel;
  public double YAccel;

  public double YawAccel;

  public Measurement(int FLE, int FRE, int RLE, int RRE, ADIS16448_IMU IMU) {}
}
