package frc.robot.subsystems.cartridge;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class CartridgeIOSim implements CartridgeIO {
  private final DCMotorSim cartridge = new DCMotorSim(DCMotor.getNeo550(1), 1.0, 1.0);
  private final DIOSim beamOne = new DIOSim(0);
  private final DIOSim beamTwo = new DIOSim(1);
  private double output = 0.0;

  @Override
  public void updateInputs(CartridgeIOInputs inputs) {
    cartridge.update(.02);

    inputs.beamOne = beamOne.getValue();
    inputs.beamTwo = beamTwo.getValue();
    inputs.appliedVolts = output;
    inputs.temperature = 0.0;
    inputs.velocityRPM =
        cartridge.getAngularVelocityRadPerSec() * 60. / (2 * Math.PI); // Check this formula
  }

  @Override
  public void setVoltage(double volts) {
    output = volts;
    cartridge.setInput(volts);
  }
}
