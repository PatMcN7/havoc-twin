package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOTalonFX implements ShooterIO {
  private static double SHOOTER_RATIO = 1.5;

  private final TalonFX left = new TalonFX(1, "CANivore");
  private final TalonFX right = new TalonFX(0, "CANivore");

  public ShooterIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    left.getConfigurator().apply(config);
    right.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftAppliedVolts = left.getMotorVoltage().getValueAsDouble();
    inputs.leftCurrentAmps = left.getSupplyCurrent().getValueAsDouble();
    inputs.leftTemperature = left.getDeviceTemp().getValueAsDouble();
    inputs.leftVelocityRPM = left.getVelocity().getValueAsDouble() * 60 * 1.5;
  }

  @Override
  public void setRightVoltage(double volts) {
    right.setControl(new VoltageOut(volts));
  }

  @Override
  public void setLeftVoltage(double volts) {
    left.setControl(new VoltageOut(volts));
  }

  @Override
  public void setLeftRPM(double RPM) {
    left.setControl(new VelocityVoltage(RPM).withEnableFOC(true).withSlot(0));
  }

  @Override
  public void setRightRPM(double RPM) {
    right.setControl(new VelocityVoltage(RPM).withEnableFOC(true).withSlot(0));
  }

  @Override
  public void configureGains(double kS, double kV, double kP, double kI, double kD) {
    var gainConfig = new Slot0Configs();
    gainConfig.kS = kS;
    gainConfig.kV = kV;
    gainConfig.kP = kP;
    gainConfig.kI = kI;
    gainConfig.kD = kD;
    left.getConfigurator().apply(gainConfig);
    right.getConfigurator().apply(gainConfig);
  }
}
