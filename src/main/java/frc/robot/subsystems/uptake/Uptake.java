package frc.robot.subsystems.uptake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Uptake extends SubsystemBase {
  private final UptakeIO io;
  private final UptakeIOInputsAutoLogged inputs = new UptakeIOInputsAutoLogged();
  private static Uptake instance;

  public Uptake(UptakeIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
      case SIM:
    }
  }

  public static Uptake getInstance() {
    if (instance == null) {
      if (Constants.currentMode.equals(Constants.Mode.REAL)) {
        return instance = new Uptake(new UptakeIOTalonFX());
      } else if (Constants.currentMode.equals(Constants.Mode.SIM)) {
        System.out.println("Uptake works");

        return instance = new Uptake(new UptakeIOSim());
      }

      return instance;
    } else {
      return instance;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Uptake", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public boolean getBeamBreak() {
    return inputs.beamBreak;
  }
}
