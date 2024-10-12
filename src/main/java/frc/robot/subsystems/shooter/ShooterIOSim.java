package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private FlywheelSim left = new FlywheelSim(DCMotor.getKrakenX60(1), 1.0, 1.);
  private FlywheelSim right = new FlywheelSim(DCMotor.getKrakenX60(1), 1.0, 1.);
  private PIDController leftPID = new PIDController(0, 0, 0);
  private PIDController rightPID = new PIDController(0, 0, 0);

  private boolean isClosedLoop = false;
  private double leftOut = 0.0;
  private double rightOut = 0.0;

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
    isClosedLoop = true;
    leftPID.setSetpoint(leftRPM);
    rightPID.setSetpoint(rightRPM);
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
