// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class FollowPath extends Command {
  /** Creates a new FollowPath. */

  private Drive drive;
  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;
  private static final PIDController ROTATION_PID_CONTROLLER;
  private static final PIDController TRANSLATION_PID_CONTROLLER;



  public FollowPath(Drive drive, PathPlannerPath path) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.path = path;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ROTATION_PID_CONTROLLER.reset();
    TRANSLATION_PID_CONTROLLER.reset();
    trajectory = new PathPlannerTrajectory(path, drive.getChassisSpeeds(), path.getStartingDifferentialPose().getRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
