package frc.robot.control;

import edu.wpi.first.math.Matrix;

public class DrivetrainControl {
  private Measurement previousEstimate;
  private Measurement current;
  private Measurement estimate;

  Matrix R;
  Matrix Q;

  /*
   * R: Covariance matrix for IMU noise and encoders
   * Q: Covariance matrix for motor powers, velocity, acceleration, position, rotation
   *
   * k = F * k-1 (momentum, drag, & gravity contribute to position) + (Motor powers contributing to velocity) + (random noise?)
   *
   */

  public float[] position;
  public float[] velocity;
  public float[] acceleration;

  public void update(Measurement newMeasurement) {
    current = newMeasurement;
    previousEstimate = estimate;
    estimate = estimate(current, predict(), previousEstimate);
  }

  Measurement predict() {
    return null;
  }

  Measurement estimate(Measurement actual, Measurement predicted, Measurement previousEstimate) {
    return null;
  }

  Matrix stateEvolution(Matrix state, float delta) {
    // acceleration varies from powered movement, drag(?), and possible noise (hitting a wall)

    // velocity increases linearly from acceleration
    for (int i = 0; i < 3; i++) {
      state.set(1, i, state.get(0, i) * (1 + delta));
    }

    // position increases linearly from velocity
    for (int i = 0; i < 3; i++) {
      state.set(2, i, state.get(1, i) * (1 + delta));
    }

    return state;
  }
}
