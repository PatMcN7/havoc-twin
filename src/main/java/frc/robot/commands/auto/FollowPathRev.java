// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.shooter.rev;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class FollowPathRev extends ParallelDeadlineGroup {

  /** Creates a new FollowPathRev. */
  public FollowPathRev(
      Drive drive, String name, boolean reset, Shooter shooter, double leftRPM, double rightRPM) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().

    super(drive.followPath(name, reset), new rev(shooter, leftRPM, rightRPM));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
