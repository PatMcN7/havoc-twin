package frc.robot.subsystems.uptake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class UptakeIOTalonFX implements UptakeIO {
  private final TalonFX uptake = new TalonFX(0);
  private final DigitalInput beamDIO = new DigitalInput(0);

  public UptakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    uptake.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(UptakeIOInputs inputs) {
    inputs.appliedVolts = uptake.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = uptake.getSupplyCurrent().getValueAsDouble();
    inputs.temperature = uptake.getDeviceTemp().getValueAsDouble();
    inputs.beamBreak = beamDIO.get();
  }

  @Override
  public void setVoltage(double volts) {
    uptake.setControl(new VoltageOut(volts));
  }
}
