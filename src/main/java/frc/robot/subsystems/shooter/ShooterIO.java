package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double leftAppliedVolts = 0.0;
    public double leftVelocityRPM = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTemperature = 0.0;

    public double rightAppliedVolts = 0.0;
    public double rightVelocityRPM = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTemperature = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double leftVolts, double rightVolts) {}

  public default void setRPM(double leftRPM, double rightRPM) {}

  public default void configureGains(double kS, double kV, double kP, double kI, double kD) {}
}
