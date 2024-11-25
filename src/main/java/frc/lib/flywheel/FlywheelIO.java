package frc.lib.flywheel;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface representing the Flywheel Input/Output (IO) operations. This interface defines methods
 * for controlling and monitoring a flywheel system, including setting voltages, velocities, and PID
 * values, as well as updating sensor inputs.
 */
public interface FlywheelIO {

  /**
   * Class containing the input data for the flywheel system. This includes the current temperature,
   * velocity, applied voltage, and current for the flywheel.
   */
  @AutoLog
  public class FlywheelIOInputs {

    /** Temperature of the flywheel system in degrees Celsius. */
    double temperatureCelsius;

    /** Velocity of the flywheel in RPM (Revolutions Per Minute). */
    double velocityRPM;

    /** Applied voltage to the flywheel system in Volts. */
    double appliedVolts;

    /** Current drawn by the flywheel system in Amps. */
    double currentAmps;

    String name;
  }

  /**
   * Sets the voltage to the flywheel system.
   *
   * @param volts The voltage to be applied to the flywheel. A positive value will spin the flywheel
   *     forward, while a negative value will spin it in reverse.
   */
  public default void setVoltage(double volts) {}

  /**
   * Updates the current inputs for the flywheel system.
   *
   * @param inputs The {@link FlywheelIOInputs} object containing the latest input data for the
   *     flywheel, such as temperature, velocity, voltage, and current.
   */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /**
   * Sets the target velocity for the flywheel system.
   *
   * @param RPM The desired velocity in Revolutions Per Minute (RPM). A positive value will cause
   *     the flywheel to spin forward.
   */
  public default void setVelocity(double RPM) {}

  /** Stops the flywheel by setting the velocity to zero and/or cutting off the voltage. */
  public default void stop() {}

  /**
   * Sets the PID and feedforward constants for controlling the flywheel.
   *
   * @param kP Proportional gain constant for the PID controller.
   * @param kI Integral gain constant for the PID controller.
   * @param kD Derivative gain constant for the PID controller.
   * @param FF Feedforward constant for the flywheel control.
   * @param kS Static gain constant for the flywheel control.
   * @param kV Velocity gain constant for the flywheel control.
   */
  public default void setPID(double kP, double kI, double kD, double FF, double kS, double kV) {}
}
