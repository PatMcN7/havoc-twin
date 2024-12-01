// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private static Vision instance;

  public static Vision getInstance() {
    if (instance == null) {

      if (Constants.currentMode.equals(Constants.Mode.REAL)) {
        return instance = new Vision(new VisionIOPhoton());
      } else if (Constants.currentMode.equals(Constants.Mode.SIM)) {
        return instance = new Vision(new VisionIOSim());
      } else {
        return instance = new Vision(new VisionIO() {});
      }
    } else {
      return instance;
    }
  }

  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    RobotState.updateVision(inputs.fieldRelativePose.toPose2d());
  }

  public void switchPipeline(int index) {
    io.switchPipeline(index);
  }
}
