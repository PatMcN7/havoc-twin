// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.beltwrap.Beltwrap;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.uptake.Uptake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowPathIntake extends ParallelDeadlineGroup {
  /** Creates a new FollowPathIntake. */
  private Drive drive;

  public FollowPathIntake(Drive drive, String path, boolean reset) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
        new Intake(
            Uptake.getInstance(),
            Beltwrap.getInstance(),
            Cartridge.getInstance(),
            Arm.getInstance(),
            35.0),
        drive.followPath(path, reset));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
