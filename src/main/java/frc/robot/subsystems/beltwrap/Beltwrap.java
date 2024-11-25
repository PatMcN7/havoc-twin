// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beltwrap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Beltwrap extends SubsystemBase {
  private final BeltwrapIO io;
  private final BeltwrapIOInputsAutoLogged inputs = new BeltwrapIOInputsAutoLogged();
  private static Beltwrap instance;
  /** Creates a new Beltwrap. */
  public Beltwrap(BeltwrapIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
      case SIM:
    }
  }

  public static Beltwrap getInstance() {
    if (instance == null) {
      if (Constants.currentMode.equals(Constants.Mode.REAL)) {
        return instance = new Beltwrap(new BeltwrapIOSparkMax());
      } else if (Constants.currentMode.equals(Constants.Mode.SIM)) {
        System.out.println("Beltwrap works");

        return instance = new Beltwrap(new BeltwrapIOSim());
      } else if (Constants.currentMode.equals(Constants.Mode.REPLAY)) {
        return instance = new Beltwrap(new BeltwrapIO() {});
      } else {
        return instance;
      }
    } else {
      return instance;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Beltwrap", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }
}
