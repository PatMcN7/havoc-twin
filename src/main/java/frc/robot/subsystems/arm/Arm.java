// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(0, 0, 0, 0, 0, 0);
        io.configureMotionMagic(0, 0, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void setPosition(double position) {
    io.setPosition(position);
    Logger.recordOutput("Arm/setpoint", position);
  }

  public void setNeutralMode(NeutralModeValue value) {
    io.configureNeutralMode(value);
  }
}