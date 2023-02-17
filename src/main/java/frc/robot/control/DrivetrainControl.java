package frc.robot.control;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N11;

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

  Matrix<N11, N1> previousFiltered;
  Matrix<N11, N1> state;

  public void initialize() {
    // 3d position + 3d velocity + 3d acceleration + 2d rotation
    state = new Matrix<N11, N1>(N11.instance, N1.instance);
    previousFiltered = null;
  }

  public void update(Matrix<N11, N1> newMeasurement, double delta) {
    Matrix<N11, N1> predicted =
        stateEvolution(previousFiltered, delta).plus(controlEvolution(previousFiltered));
    Matrix<N11, N11> predictedCovariance = StateSpaceUtil.makeCovarianceMatrix(N11.instance, null);
  }

  Measurement predict() {
    return null;
  }

  Measurement estimate(Measurement actual, Measurement predicted, Measurement previousEstimate) {
    return null;
  }

  Matrix<N11, N1> stateEvolution(Matrix<N11, N1> state, double delta) {
    // acceleration varies from powered movement, drag(?), and possible noise (hitting a wall)

    // velocity increases linearly from acceleration
    for (int i = 0; i < 3; i++) {
      state.set(3 + i, 0, state.get(i, 0) * (1 + delta));
    }

    // position increases linearly from velocity
    for (int i = 0; i < 3; i++) {
      state.set(6 + i, 0, state.get(3 + i, 0) * (1 + delta));
    }

    return state;
  }

  Matrix<N11, N1> controlEvolution(Matrix<N11, N1> control) {
    return control;
  }
}
