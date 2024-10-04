package frc.robot.subsystems.cartridge;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Cartridge extends SubsystemBase {
  private final CartridgeIO io;
  private final CartridgeIOInputsAutoLogged inputs = new CartridgeIOInputsAutoLogged();

  public Cartridge(CartridgeIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
      case SIM:
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Cartridge", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public boolean getFirstBeam() {
    return inputs.beamOne;
  }

  public boolean getSecondBeam() {
    return inputs.beamTwo;
  }
}
