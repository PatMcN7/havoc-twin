package frc.robot.subsystems.beltwrap;

import org.littletonrobotics.junction.AutoLog;

public interface BeltwrapIO {
  @AutoLog
  public static class BeltwrapIOInputs {
    public double temperature = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(BeltwrapIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
