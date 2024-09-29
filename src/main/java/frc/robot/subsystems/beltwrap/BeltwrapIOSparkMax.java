package frc.robot.subsystems.beltwrap;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class BeltwrapIOSparkMax implements BeltwrapIO {
  private final CANSparkMax beltwrap = new CANSparkMax(0, MotorType.kBrushless);

  public BeltwrapIOSparkMax() {
    beltwrap.restoreFactoryDefaults();
    beltwrap.setCANTimeout(250);
    beltwrap.setInverted(false);
    beltwrap.setSmartCurrentLimit(30);
    beltwrap.burnFlash();
  }

  @Override
  public void updateInputs(BeltwrapIOInputs inputs) {
    inputs.appliedVolts = beltwrap.getAppliedOutput();
    inputs.currentAmps = beltwrap.getOutputCurrent();
    inputs.temperature = beltwrap.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    beltwrap.setVoltage(volts);
  }
}
