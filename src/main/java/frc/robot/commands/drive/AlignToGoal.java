// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlignToGoal extends Command {
  /** Creates a new AlignToGoal. */
  private Drive drive;

  private Command pathFindCommand;
  private DoubleSupplier toGoal;

  public AlignToGoal(Drive drive, DoubleSupplier toGoal) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.toGoal = toGoal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathFindCommand = drive.alignToGoal(toGoal);
    pathFindCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("Is Pathfind Command Scheduled?", pathFindCommand.isScheduled());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Is Pathfind Command Scheduled?", pathFindCommand.isScheduled());
    pathFindCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.recordOutput(
        "Is rot values Near?",
        MathUtil.isNear(toGoal.getAsDouble(), drive.getPose().getRotation().getRadians(), 0.5));
    return MathUtil.isNear(toGoal.getAsDouble(), drive.getPose().getRotation().getRadians(), 0.05);
  }
}
