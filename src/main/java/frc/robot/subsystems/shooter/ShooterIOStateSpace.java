package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.Logger;

public class ShooterIOStateSpace implements ShooterIO {
  private FlywheelSim left = new FlywheelSim(DCMotor.getKrakenX60(1), 1.0, 0.0058);
  private FlywheelSim right = new FlywheelSim(DCMotor.getKrakenX60(1), 1.0, 0.0058);
  private PIDController leftPID = new PIDController(0, 0, 0);
  private PIDController rightPID = new PIDController(0, 0, 0);

  private boolean isClosedLoop = false;
  private double leftOut = 0.0;
  private double rightOut = 0.0;

  private final LinearSystem<N1, N1, N1> leftFlywheelPlant =
      LinearSystemId.identifyVelocitySystem(0.002, 0.001);
  private final LinearSystem<N1, N1, N1> rightFlywheelPlant =
      LinearSystemId.identifyVelocitySystem(0.002, 0.001);
  private final KalmanFilter<N1, N1, N1> leftObserver =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          leftFlywheelPlant,
          VecBuilder.fill(3000.0), // How accurate we think our model is
          VecBuilder.fill(.01), // How accurate we think our encoder
          // data is
          0.020);
  private final KalmanFilter<N1, N1, N1> rightObserver =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          rightFlywheelPlant,
          VecBuilder.fill(3000.0), // How accurate we think our model is
          VecBuilder.fill(.01), // How accurate we think our encoder
          // data is
          0.020);

  private final LinearQuadraticRegulator<N1, N1, N1> leftController =
      new LinearQuadraticRegulator<>(
          leftFlywheelPlant,
          VecBuilder.fill(6.0), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave
          // more
          // aggressively.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12
          // is a good
          // starting point because that is the (approximate) maximum voltage of a
          // battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  private final LinearQuadraticRegulator<N1, N1, N1> rightController =
      new LinearQuadraticRegulator<>(
          rightFlywheelPlant,
          VecBuilder.fill(6.0), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave
          // more
          // aggressively.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12
          // is a good
          // starting point because that is the (approximate) maximum voltage of a
          // battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  private final LinearSystemLoop<N1, N1, N1> leftLoop =
      new LinearSystemLoop<>(leftFlywheelPlant, leftController, leftObserver, 12.0, 0.020);

  private final LinearSystemLoop<N1, N1, N1> rightLoop =
      new LinearSystemLoop<>(rightFlywheelPlant, rightController, rightObserver, 12.0, 0.020);

  public ShooterIOStateSpace() {
    leftLoop.reset(VecBuilder.fill(left.getAngularVelocityRPM()));
    rightLoop.reset(VecBuilder.fill(right.getAngularVelocityRPM()));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (isClosedLoop) {
      leftOut = leftPID.calculate(left.getAngularVelocityRPM());
      rightOut = rightPID.calculate(right.getAngularVelocityRPM());
      left.setInput(leftOut);
      right.setInput(rightOut);
    }

    left.update(.02);
    right.update(.02);

    inputs.leftTemperature = 0.0;
    inputs.rightTemperature = 0.0;
    inputs.leftCurrentAmps = left.getCurrentDrawAmps();
    inputs.rightCurrentAmps = right.getCurrentDrawAmps();
    inputs.leftVelocityRPM = left.getAngularVelocityRPM();
    inputs.rightVelocityRPM = right.getAngularVelocityRPM();
    inputs.leftAppliedVolts = leftOut;
    inputs.rightAppliedVolts = rightOut;

    leftLoop.correct(VecBuilder.fill(left.getAngularVelocityRPM()));

    rightLoop.correct(VecBuilder.fill(right.getAngularVelocityRPM()));
    leftLoop.predict(0.02);
    rightLoop.predict(0.02);

    double leftVoltage;
    double rightVoltage;
    leftVoltage = leftLoop.getU(0);
    rightVoltage = rightLoop.getU(0);
    Logger.recordOutput("Shooter/Left Voltage SS", leftVoltage);
    Logger.recordOutput("Shooter/Right Voltage Statespace", rightVoltage);
    left.setInput(MathUtil.clamp(leftVoltage, 0.0, 12.0));
    right.setInput(MathUtil.clamp(rightVoltage, 0.0, 12.0));
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    isClosedLoop = false;
    leftOut = leftVolts;
    rightOut = rightVolts;
    left.setInput(leftVolts);
    right.setInput(rightVolts);
  }

  @Override
  public void setRPM(double leftRPM, double rightRPM) {
    leftLoop.setNextR(leftRPM);
    rightLoop.setNextR(rightRPM);
  }

  @Override
  public void configureGains(double kS, double kV, double kP, double kI, double kD) {
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);

    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);
  }
}
