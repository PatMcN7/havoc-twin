package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public class ArmIOInputs {
    public double postionDeg = 0.0;
    public double velocityDegPerSec = 0.0;
    public double voltageOut = 0.0;
    public double currentAmps = 0.0;
    public double temperature = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setPosition(double position) {}
}
