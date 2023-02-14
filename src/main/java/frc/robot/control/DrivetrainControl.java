package frc.robot.control;

public class DrivetrainControl {
  private int FLEncoder;
  private int FREncoder;
  private int RLEncoder;
  private int RREncoder;

  private Measurement Kminus;
  private Measurement K;
  private Measurement Kplus;

  public void update(Measurement newMeasurement) {
    Kminus = K;
    K = newMeasurement;
    Kplus = predict();
  }

  Measurement predict() {
    return null;
  }
}
