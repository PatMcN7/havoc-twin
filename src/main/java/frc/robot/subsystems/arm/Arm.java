// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private static Arm instance;
  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;
  private final InterpolatingDoubleTreeMap distanceToShot;
  private final InterpolatingDoubleTreeMap tyToDistance;
  public boolean zeroedFlag = false;
  public static final LoggedTunableNumber tunedAngle =
      new LoggedTunableNumber("Arm/Tuned Setpoint", 0);
  private double tunedReuslt = 0.0;
  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    distanceToShot = new InterpolatingDoubleTreeMap();
    tyToDistance = new InterpolatingDoubleTreeMap();
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
        // io.configurePID(.75, .12, 0, 7., 0, 0);
        // io.configureMotionMagic(60.0, 60.0, 60.0);
    }

    measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);
  }

  public static Arm getInstance() {
    if (instance == null) {
      if (Constants.currentMode.equals(Constants.Mode.REAL)) {
        return instance = new Arm(new ArmIOTalonFX());
      } else if (Constants.currentMode.equals(Constants.Mode.SIM)) {
        System.out.println("Arm works");
        return instance = new Arm(new ArmIOSim());

      } else {
        return instance;
      }
    } else {
      return instance;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (inputs.atZero) {
      zeroedFlag = true;
      zeroArm();
    }

    measuredVisualizer.update(inputs.postionDeg);
    setpointVisualizer.update(inputs.setpointDeg);

    Logger.recordOutput("Tuned Number Output", tunedAngle.get());
  }

  public void setTunedPos() {
    LoggedTunableNumber.ifChanged(hashCode(), () -> setPosition(tunedAngle.get()), tunedAngle);
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

  public void zeroArm() {
    io.zeroArm();
  }

  public boolean atPosition() {
    return inputs.atPosition;
  }

  public boolean isArmZeroed() {
    return inputs.atZero;
  }

  public InstantCommand zeroCommand() {
    return new InstantCommand(() -> io.setVoltage(-0.4), Arm.getInstance());
  }

  public double getShotAngle() {
    return distanceToShot.get(tyToDistance.get(LimelightHelpers.getTY("")));
  }
}
