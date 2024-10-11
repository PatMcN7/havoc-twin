package frc.robot.subsystems.uptake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class UptakeIOSim implements UptakeIO {
  private DCMotorSim uptake = new DCMotorSim(DCMotor.getNEO(1), 1., 1.);
  private DIOSim beam = new DIOSim(3);
  private double output = 0;

  @Override
  public void updateInputs(UptakeIOInputs inputs) {
    uptake.update(.02);

    inputs.beamBreak = beam.getValue();
    inputs.currentAmps = uptake.getCurrentDrawAmps();
    inputs.temperature = 0.0;
    inputs.appliedVolts = output;
  }

  @Override
  public void setVoltage(double volts) {
    uptake.setInput(volts);
    output = volts;
  }
}
