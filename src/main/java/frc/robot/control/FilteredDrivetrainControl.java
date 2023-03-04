package frc.robot.control;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import org.ejml.simple.SimpleMatrix;

public class FilteredDrivetrainControl extends CommandBase {
  // 0-1 acceleration, 2-3 velocity, 4-5 position, 6-7 2d rotation vector, 8 yaw velocity
  private N3 state;
  // 0 mecanum x velocity, 1 mecanum y velocity, 2 yaw velocity
  private N3 input;
  // 0-2 imu accelerometer, 3-4 2d rotation, 5 yaw accel, 6-7 x-y encoder change (robot space)
  private N3 measurements;

  private KalmanFilter<N3, N3, N3> filter;
  private Matrix<N3, N1> controlVector;
  private double[] estimatedStates;
  private DriveSubsystem drivetrain;
  private PIDController xPID;
  private PIDController yPID;
  private PIDController zRotPID;
  private Timer timer;
  private double storedTime;

  private ADIS16470_IMU imuRef;

  public FilteredDrivetrainControl(ADIS16470_IMU imu) {
    imuRef = imu;
  }

  public void initialize() {
    // accel x y z (z altitude)
    // 0.012532, 0.014077, 0.025190
    double[][] stdDev = {{0.012532}, {0.014077}, {0.025190}};
    double[][] stateDev = {{0.001}, {0.001}, {0.001}};
    Matrix<N3, N1> sensorDeviations = new Matrix<N3, N1>(new SimpleMatrix(stdDev));
    Matrix<N3, N1> stateDeviations = new Matrix<N3, N1>(new SimpleMatrix(stateDev));
    // identity matrices because testing
    double[][] a = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
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
    xPID = new PIDController(0.1, 0.3, 0.3);
    yPID = new PIDController(0.1, 0.3, 0.3);
    zRotPID = new PIDController(0.1, 0.3, 0.3);
    double[][] empty = new double[3][1];
    double[][] bruh = {{0}, {0}, {0}};
    controlVector = new Matrix<N3, N1>(new SimpleMatrix(empty));

    filter.setXhat(new Matrix<N3, N1>(new SimpleMatrix(bruh)));

    timer = new Timer();
    timer.start();
  }

  public void execute(boolean log) {
    double dT = timer.get() - storedTime;
    filter.predict(controlVector, dT);
    filter.correct(controlVector, getMeasurements());

    storedTime = timer.get();
    controlVector = getControlVector(filter.getXhat());

    if (log) {
      System.out.println("Filter data:");
      System.out.println(filter.getXhat());
      System.out.println(controlVector);
    }
    // drivetrain.drive(controlVector.get(0, 0), controlVector.get(1, 0), controlVector.get(2, 0));
  }

  private Matrix<N3, N1> getControlVector(Matrix<N3, N1> state) {
    Matrix<N3, N1> controlVector = new Matrix<N3, N1>(N3.instance, N1.instance);

    // controlVector.set(0, 0, xPID.calculate(state.get(0, 0), 0));
    // controlVector.set(1, 0, yPID.calculate(state.get(1, 0), 0));
    // controlVector.set(2, 0, zRotPID.calculate(state.get(2, 0), 0));

    return controlVector;
  }

  private Matrix<N3, N1> getMeasurements() {
    Matrix<N3, N1> measurementVector = new Matrix<N3, N1>(N3.instance, N1.instance);
    // double[] rawData = drivetrain.getIMUData();
    double[][] columnVec = {
      {imuRef.getAccelX() + 0.29}, {imuRef.getAccelY() + 0.29}, {imuRef.getAccelZ()}
    };
    measurementVector = new Matrix<N3, N1>(new SimpleMatrix(columnVec));
    return measurementVector;
  }

  private void integratePosVel() {}
}
