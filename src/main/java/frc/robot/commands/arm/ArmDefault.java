// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.cartridge.Cartridge;

public class ArmDefault extends Command {
  private Arm arm;
  private boolean pieceIn;
  /** Creates a new ArmDefault. */
  public ArmDefault(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    pieceIn = Cartridge.hasPiece();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pieceIn = Cartridge.hasPiece();
    if (pieceIn) {
      arm.setPosition(0);
    } else {
      arm.setPosition(35.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
