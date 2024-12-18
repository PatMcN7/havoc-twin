package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private static double SHOOTER_RATIO = 1.5;

  private final TalonFX left = new TalonFX(19, "CANIVORE 3");
  private final TalonFX right = new TalonFX(18, "CANIVORE 3");

  public ShooterIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    left.getConfigurator().apply(config);
    right.getConfigurator().apply(config);
    left.setInverted(false);
    right.setInverted(true);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftAppliedVolts = left.getMotorVoltage().getValueAsDouble();
    inputs.leftCurrentAmps = left.getSupplyCurrent().getValueAsDouble();
    inputs.leftTemperature = left.getDeviceTemp().getValueAsDouble();
    inputs.leftVelocityRPM = left.getVelocity().getValueAsDouble() * 60 * 1.5;
    inputs.leftAtVelocity = MathUtil.isNear(0.0, left.getClosedLoopError().getValueAsDouble(), 1.5);
    inputs.rightAtVelocity =
        MathUtil.isNear(0.0, right.getClosedLoopError().getValueAsDouble(), 1.5);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    left.setControl(new VoltageOut(leftVolts));
    right.setControl(new VoltageOut(rightVolts));
  }

  @Override
  public void setRPM(double leftRPM, double rightRPM) {
    left.setControl(
        new VelocityVoltage(leftRPM / 60.)
            .withEnableFOC(false)
            .withSlot(0)
            .withLimitReverseMotion(true));
    right.setControl(
        new VelocityVoltage(rightRPM / 60.)
            .withEnableFOC(false)
            .withSlot(0)
            .withLimitReverseMotion(true));
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
