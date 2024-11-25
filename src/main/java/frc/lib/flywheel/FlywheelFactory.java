package frc.lib.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.HashMap;
import java.util.Map;

public class FlywheelFactory {
  private static Map<String, Flywheel> flywheels = new HashMap<>();

  public static Flywheel getFlywheel(String name) {
    if (flywheels.containsKey(name)) {
      return flywheels.get(name);
    } else {
      flywheels.put(name, new Flywheel(new FlywheelIOSim(name)));
      return flywheels.get(name);
    }
  }

  public static Flywheel getFlywheel(String name, DCMotor motor, double gearing, double MOI) {
    if (flywheels.containsKey(name)) {
      return flywheels.get(name);
    } else {
      flywheels.put(name, new Flywheel(new FlywheelIOSim(motor, gearing, MOI, name)));
      return flywheels.get(name);
    }
  }

  public static void makeFlywheel(String name, DCMotor motor, double gearing, double MOI) {
    flywheels.put(name, new Flywheel(new FlywheelIOSim(name)));
  }
}
