// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private String name;
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(inputs.name, inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runVelocity(double RPM) {
    io.setVelocity(RPM);
  }

  public void setPID(double kP, double kI, double kD) {
    // inputs.set
  }
}
