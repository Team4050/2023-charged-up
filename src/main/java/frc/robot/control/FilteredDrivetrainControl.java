package frc.robot.control;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ejml.simple.SimpleMatrix;

public class FilteredDrivetrainControl extends CommandBase {
  private KalmanFilter<N3, N3, N3> filter;
  private Matrix<N3, N1> controlVector;

  public FilteredDrivetrainControl() {}

  public void initialize() {
    // accel x y z (z altitude)
    // 0.012532, 0.014077, 0.025190
    double[][] stdDev = {{0.02}, {0.02}, {0.05}};
    double[][] stateDev = {{0.01}, {0.01}, {0.001}};
    Matrix<N3, N1> sensorDeviations = new Matrix<N3, N1>(new SimpleMatrix(stdDev));
    Matrix<N3, N1> stateDeviations = new Matrix<N3, N1>(new SimpleMatrix(stateDev));
    // identity matrices because testing
    double[][] a = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Matrix<N3, N3> A =
        new Matrix<N3, N3>(new SimpleMatrix(a)); // corresponds to F (state-space transition model)
    double[][] b = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Matrix<N3, N3> B =
        new Matrix<N3, N3>(new SimpleMatrix(b)); // corresponds to B (input transform model)
    double[][] c = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Matrix<N3, N3> C =
        new Matrix<N3, N3>(new SimpleMatrix(c)); // corresponds to H (state observation model)
    double[][] d = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Matrix<N3, N3> D =
        new Matrix<N3, N3>(
            new SimpleMatrix(
                d)); // feedforward control matrix (zero matrix because of no feedforward?)
    LinearSystem<N3, N3, N3> system = new LinearSystem<N3, N3, N3>(A, B, C, D);
    filter =
        new KalmanFilter<N3, N3, N3>(
            N3.instance, N3.instance, system, stateDeviations, sensorDeviations, 0.05);
    double[][] empty = new double[3][1];
    double[][] zero = {{0}, {0}, {0}};
    controlVector = new Matrix<N3, N1>(new SimpleMatrix(empty));
    filter.setXhat(new Matrix<N3, N1>(new SimpleMatrix(zero)));
  }

  public void execute(double dT, double[][] data) {
    filter.predict(controlVector, dT);
    filter.correct(controlVector, new Matrix<N3, N1>(new SimpleMatrix(data)));
  }

  public Matrix<N3, N1> getStateEstimate() {
    return filter.getXhat();
  }

  public Matrix<N3, N1> getControlVector() {
    return controlVector;
  }
}
