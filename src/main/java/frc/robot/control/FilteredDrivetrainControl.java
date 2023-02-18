package frc.robot.control;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N10;
import edu.wpi.first.math.numbers.N11;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N9;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class FilteredDrivetrainControl extends CommandBase {
  // 0-1 position, 2-3 velocity, 4-5 acceleration, 6-7 2d rotation vector, 8 yaw velocity
  private N11 state;
  // 0 mecanum x velocity, 1 mecanum y velocity, 2 yaw velocity
  private N3 input;
  // 0-2 imu accelerometer, 3-4 2d rotation, 5 yaw accel, 6-7 x-y encoder change (robot space)
  private N10 measurements;

  private KalmanFilter<N11, N3, N10> filter;
  private Matrix<N3, N1> controlVector;
  private DriveSubsystem drivetrain;
  private PIDController xPID;
  private PIDController yPID;
  private PIDController zRotPID;
  private Timer timer;
  private double storedTime;

  public void initialize() {
    Matrix<N10, N1> sensorDeviations = null;
    Matrix<N11, N1> stateDeviations = null;
    Matrix<N11, N11> A = null;
    Matrix<N11, N3> B = null;
    Matrix<N10, N11> C = null;
    Matrix<N3, N10> D = null;
    LinearSystem<N11, N3, N9> system = new LinearSystem(A, B, C, D);
    filter = new KalmanFilter(state, input, system, stateDeviations, sensorDeviations, 0);

    xPID = new PIDController(0.5, 0.5, 1);
    yPID = new PIDController(0.5, 0.5, 1);
    zRotPID = new PIDController(0.5, 0.5, 1);

    timer = new Timer();
    timer.start();
  }

  @Override
  public void execute() {
    double dT = timer.get() - storedTime;
    filter.predict(controlVector, dT);
    filter.correct(controlVector, getMeasurements());

    storedTime = timer.get();
    controlVector = getControlVector(filter.getXhat());

    drivetrain.drive(controlVector.get(0, 0), controlVector.get(1, 0), controlVector.get(2, 0));
  }

  private Matrix<N3, N1> getControlVector(Matrix<N11, N1> state) {
    Matrix<N3, N1> controlVector = new Matrix<N3, N1>(N3.instance, N1.instance);

    controlVector.set(0, 0, xPID.calculate(state.get(2, 0)));
    controlVector.set(1, 0, yPID.calculate(state.get(3, 0)));
    controlVector.set(2, 0, zRotPID.calculate(state.get(8, 0)));

    return controlVector;
  }

  private Matrix<N10, N1> getMeasurements() {
    Matrix<N10, N1> measurementVector = new Matrix<N10, N1>(N10.instance, N1.instance);
    double[] rawData = drivetrain.getIMUData();

    for (int i = 0; i < rawData.length; i++) {
      measurementVector.set(i, 0, rawData[i]);
    }

    measurementVector.set(3, 0, rawData[3]);
    measurementVector.set(4, 0, rawData[4]);

    measurementVector.set(5, 0, rawData[5]);

    return measurementVector;
  }
}
