package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class ArmIOTalonFX implements ArmIO {
  private static final double GEAR_RATIO = 47.6;
  private final TalonFX arm = new TalonFX(28, "CANIVORE 3");
  private final MotionMagicVoltage mRequest = new MotionMagicVoltage(0);

  public ArmIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kS = .75;
    config.Slot0.kV = .12;
    config.Slot0.kP = 7.0;
    config.MotionMagic.MotionMagicAcceleration = 60.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 60.0;

    arm.getConfigurator().apply(config);
    arm.setPosition(0.0);
    arm.setInverted(false);
  }

  public double degreesToRotations(double degrees) {
    return Units.degreesToRotations(degrees * GEAR_RATIO);
  }

  public double rotationsToDegrees(double rotations) {
    return Units.rotationsToDegrees(rotations / GEAR_RATIO);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.currentAmps = arm.getStatorCurrent().getValueAsDouble();
    inputs.postionDeg = rotationsToDegrees(arm.getPosition().getValueAsDouble());
    inputs.temperature = arm.getDeviceTemp().getValueAsDouble();
    inputs.voltageOut = arm.getMotorVoltage().getValueAsDouble();
    inputs.velocityDegPerSec = arm.getVelocity().getValueAsDouble() * GEAR_RATIO;
    inputs.atPosition = atPosition();
    inputs.setpoint = arm.getClosedLoopReference().getValueAsDouble();
    Logger.recordOutput(
        "Arm Rotations To Degrees", Units.rotationsToDegrees(arm.getPosition().getValueAsDouble()));
    Logger.recordOutput("Motor Rotations", arm.getPosition().getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    arm.setControl(new VoltageOut(volts));
  }

  @Override
  public void setPosition(double position) {
    // Position here is in degrees
    Logger.recordOutput("Other Arm Setpoint", position);
    Logger.recordOutput(
        "Arm setpoint clamped and converted",
        MathUtil.clamp(degreesToRotations(position), 0.0, 80.));
    arm.setControl(mRequest.withPosition(MathUtil.clamp(degreesToRotations(position), 0.0, 80.)));
  }

  @Override
  public void configurePID(double kS, double kV, double kA, double kP, double kI, double kD) {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;
    slot0Configs.kP = kP;
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

  @Override
  public void zeroArm() {
    arm.setPosition(0.0);
  }

  public boolean atPosition() {
    if (MathUtil.isNear(0., rotationsToDegrees(arm.getClosedLoopError().getValueAsDouble()), 2.)) {
      return true;
    } else {
      return false;
    }
  }
}
