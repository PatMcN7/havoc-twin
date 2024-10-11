package frc.robot.subsystems.beltwrap;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class BeltwrapIOSim implements BeltwrapIO {
  private final DCMotorSim beltwrap = new DCMotorSim(DCMotor.getNEO(1), 1, 1.0);
  private double output = 0.0;

  @Override
  public void updateInputs(BeltwrapIOInputs inputs) {
    beltwrap.update(.02);

    inputs.appliedVolts = output;
    inputs.currentAmps = beltwrap.getCurrentDrawAmps();
    inputs.temperature = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    output = volts;
    beltwrap.setInput(volts);
  }
}
