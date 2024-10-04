package frc.robot.subsystems.cartridge;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class CartridgeIOSparkMax implements CartridgeIO {
  private final CANSparkMax cartridge = new CANSparkMax(0, MotorType.kBrushless);
  private final DigitalInput firstBeam = new DigitalInput(0);
  private final DigitalInput secondBeam = new DigitalInput(1);

  public CartridgeIOSparkMax() {
    cartridge.restoreFactoryDefaults();
    cartridge.setCANTimeout(250);
    cartridge.setInverted(false);
    cartridge.setSmartCurrentLimit(30);
    cartridge.burnFlash();
  }

  @Override
  public void updateInputs(CartridgeIOInputs inputs) {
    inputs.appliedVolts = cartridge.getAppliedOutput();
    inputs.currentAmps = cartridge.getOutputCurrent();
    inputs.temperature = cartridge.getMotorTemperature();
    inputs.beamOne = firstBeam.get();
    inputs.beamTwo = secondBeam.get();
  }

  @Override
  public void setVoltage(double volts) {
    cartridge.setVoltage(volts);
  }
}
