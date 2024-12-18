// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class rev extends Command {
  /** Creates a new rev. */
  private final Shooter shooter;

  private double leftRPM;
  private double rightRPM;

  public rev(Shooter shooter, double leftRPM, double rightRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leftRPM = leftRPM;
    this.rightRPM = rightRPM;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setRPM(leftRPM, rightRPM);
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
