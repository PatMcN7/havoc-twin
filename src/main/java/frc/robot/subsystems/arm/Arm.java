// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final MechanismLigament2d armLigament;
  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    Mechanism2d armSim = new Mechanism2d(1, 4);
    MechanismRoot2d armSimRoot = armSim.getRoot("Arm", 0, 0);
    armLigament = new MechanismLigament2d("Arm", 10, 180);
    armSimRoot.append(armLigament);

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

    armLigament.setAngle(new Rotation2d(inputs.postionDeg));
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
