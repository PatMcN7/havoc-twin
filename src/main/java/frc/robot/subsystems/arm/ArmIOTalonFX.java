package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;

public class ArmIOTalonFX implements ArmIO {
  private static final double GEAR_RATIO = 0.0;
  private final TalonFX arm = new TalonFX(0, "CANivore");
  private final MotionMagicVoltage mRequest = new MotionMagicVoltage(0);

  public ArmIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    arm.getConfigurator().apply(config);
    arm.setPosition(0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.currentAmps = arm.getStatorCurrent().getValueAsDouble();
    inputs.postionDeg = arm.getPosition().getValueAsDouble() * GEAR_RATIO;
    inputs.temperature = arm.getDeviceTemp().getValueAsDouble();
    inputs.voltageOut = arm.getMotorVoltage().getValueAsDouble();
    inputs.velocityDegPerSec = arm.getVelocity().getValueAsDouble() * GEAR_RATIO;
  }

  @Override
  public void setVoltage(double volts) {
    arm.setControl(new VoltageOut(volts));
  }

  @Override
  public void setPosition(double position) {

    arm.setControl(mRequest.withPosition(MathUtil.clamp(position, 0, 10000)));
  }

  @Override
  public void configurePID(double kS, double kV, double kA, double kP, double kI, double kD) {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;
    slot0Configs.kA = kA;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    arm.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
    motionMagicConfigs.MotionMagicJerk = jerk;
  }

  @Override
  public void configureNeutralMode(NeutralModeValue value) {
    arm.setNeutralMode(value);
  }
}
