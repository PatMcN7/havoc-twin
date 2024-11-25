package frc.lib.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.flywheel.FlywheelIO.FlywheelIOInputs;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim sim;
  private boolean isClosedLoop = false;
  private double appliedVolts = 0.0;
  private String name;
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  public FlywheelIOSim(DCMotor motor, double gearing, double MOI, String name) {
    sim = new FlywheelSim(motor, gearing, MOI);
    this.name = name;
  }

  public FlywheelIOSim(String name) {
    this.name = name;
  }

  public void setSimConstants(DCMotor motor, double gearing, double MOI, String name) {
    sim = new FlywheelSim(motor, gearing, MOI);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts = pid.calculate(sim.getAngularVelocityRPM());
      sim.setInputVoltage(appliedVolts);
    }
    sim.update(0.02);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.temperatureCelsius = 0.0;
    inputs.name = name;
  }

  @Override
  public void stop() {
    isClosedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setVelocity(double velocityRPM) {
    isClosedLoop = true;
    pid.setSetpoint(velocityRPM);
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPID(double kP, double kI, double kD, double FF, double kS, double kV) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }
}
