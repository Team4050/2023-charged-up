package frc.robot.control;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N11;
import org.ejml.simple.SimpleMatrix;

public class DrivetrainControl {
  private Matrix R;
  private Matrix Q;
  private Matrix I;

  /*
   * R: Covariance matrix for IMU noise and encoders
   * Q: Covariance matrix for motor powers, velocity, acceleration, position, rotation
   *
   * k = F * k-1 (momentum, drag, & gravity contribute to position) + (Motor powers contributing to velocity) + (random noise?)
   *
   */

  private Matrix<N11, N1> previousFiltered;
  private Matrix<N11, N1> state;

  private Matrix<N11, N11> previousCov;
  private Matrix<N11, N11> covariance;

  private Matrix<N11, N11> stateTransitionMatrix;
  private Matrix<N11, N11> systemObservationMatrix;

  public void initialize(float t) {
    // 3d position + 3d velocity + 3d acceleration + 2d rotation (sin, cos)
    state = new Matrix<N11, N1>(N11.instance, N1.instance);
    previousFiltered = null;
    double[][] matrixArray = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, t, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, t, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, t, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, t, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, t, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
    SimpleMatrix stateTransitionStorage = new SimpleMatrix(matrixArray);
    stateTransitionMatrix = new Matrix<N11, N11>(stateTransitionStorage);
  }

  public void update(Matrix<N11, N1> newMeasurement, double delta) {
    Matrix<N11, N1> predicted =
        stateEvolution(previousFiltered, delta).plus(controlEvolution(previousFiltered));

    predicted = diagonal(previousFiltered).times(stateTransitionMatrix).times(previousFiltered);
    Matrix<N11, N11> predictedCovariance = StateSpaceUtil.makeCovarianceMatrix(N11.instance, null);

    Matrix<N11, N11> kalmanGain = new Matrix<N11, N11>(N11.instance, N11.instance);

    covariance = (I.minus(kalmanGain.times(systemObservationMatrix)));
    state = predicted.plus(kalmanGain.times(state.minus(systemObservationMatrix.times(predicted))));
  }

  Matrix<N11, N11> diagonal(Matrix<N11, N1> vector) {
    Matrix<N11, N11> r = new Matrix<N11, N11>(N11.instance, N11.instance);
    for (int i = 0; i < 11; i++) {
      r.set(i, i, vector.get(1, 0));
    }
    return r;
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
