package frc.robot.subsystems.cartridge;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Cartridge extends SubsystemBase {
  private final CartridgeIO io;
  private final CartridgeIOInputsAutoLogged inputs = new CartridgeIOInputsAutoLogged();
  private static Cartridge instance;
  private static boolean hasPiece;

  public Cartridge(CartridgeIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
      case SIM:
    }
  }

  public static Cartridge getInstance() {
    if (instance == null) {
      if (Constants.currentMode.equals(Constants.Mode.REAL)) {
        return instance = new Cartridge(new CartridgeIOSparkMax());
      } else if (Constants.currentMode.equals(Constants.Mode.SIM)) {
        System.out.println("Cartridge works");

        return instance = new Cartridge(new CartridgeIOSim());
      } else if (Constants.currentMode.equals(Constants.Mode.REPLAY)) {
        return instance = new Cartridge(new CartridgeIO() {});
      }
      return instance;
    } else {
      return instance;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Cartridge", inputs);

    if (inputs.beamOne && inputs.beamTwo) {
      hasPiece = true;
    } else {
      hasPiece = false;
    }
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

  public static boolean hasPiece() {
    return hasPiece;
  }
}
