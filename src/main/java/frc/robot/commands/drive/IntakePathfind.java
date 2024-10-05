// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.beltwrap.Beltwrap;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.uptake.Uptake;

public class IntakePathfind extends Command {
  /** Creates a new IntakePathfind. */
  Beltwrap beltwrap;

  Uptake uptake;
  Cartridge cartridge;
  Drive drive;
  Pose2d notePose;

  public IntakePathfind(Beltwrap beltwrap, Uptake uptake, Cartridge cartridge, Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.beltwrap = beltwrap;
    this.uptake = uptake;
    this.cartridge = cartridge;
    this.drive = drive;

    addRequirements(uptake, beltwrap, cartridge, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beltwrap.runVolts(12.0);
    uptake.runVolts(12.0);
    cartridge.runVolts(6.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTargetCount(Constants.LIMELIGHT_NAME) > 0) {
      Rotation2d rotation =
          new Rotation2d(
              drive.getRotation().getDegrees() + LimelightHelpers.getTX(Constants.LIMELIGHT_NAME));
      notePose = new Pose2d(drive.getPose().getX(), drive.getPose().getY(), rotation);
      // Commands.runOnce(drive.pathfind(notePose), drive);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cartridge.getFirstBeam() || cartridge.getSecondBeam();
  }
}
