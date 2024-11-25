package frc.lib.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    public double appliedVolts;
    public double temperatureCelsius;
    public double positionMeters;
    public double velocityRadPerSec;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setPID(
      double kP, double kI, double kD, double FF, double kG, double kS, double kV) {}
}
