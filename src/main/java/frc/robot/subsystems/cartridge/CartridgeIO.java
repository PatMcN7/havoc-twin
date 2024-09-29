package frc.robot.subsystems.cartridge;

import org.littletonrobotics.junction.AutoLog;

public interface CartridgeIO {

  @AutoLog
  public static class CartridgeIOInputs {
    public double appliedVolts = 0.0;
    public double temperature = 0.0;
    public double currentAmps = 0.0;
    public double velocityRPM = 0.0;
    public boolean beamOne = false;
    public boolean beamTwo = false;
  }

  public default void updateInputs(CartridgeIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
