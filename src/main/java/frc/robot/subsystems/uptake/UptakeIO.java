package frc.robot.subsystems.uptake;

import org.littletonrobotics.junction.AutoLog;

public interface UptakeIO {
  @AutoLog
  public static class UptakeIOInputs {
    public double temperature = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean beamBreak = false;
  }

  public default void updateInputs(UptakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
