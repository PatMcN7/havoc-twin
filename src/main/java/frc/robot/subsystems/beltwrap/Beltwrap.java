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

  /** Creates a new Beltwrap. */
  public Beltwrap(BeltwrapIO io) {
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
    Logger.processInputs("Beltwrap", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }
}
