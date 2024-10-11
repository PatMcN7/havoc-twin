// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.beltwrap.Beltwrap;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.uptake.Uptake;

public class Testall extends Command {
  /** Creates a new Testall. */
  private Beltwrap beltwrap;

  private Uptake uptake;
  private Cartridge cartridge;
  private Shooter shooter;
  private Arm arm;

  public Testall(Beltwrap beltwrap, Uptake uptake, Cartridge cartridge, Shooter shooter, Arm arm) {
    this.beltwrap = beltwrap;
    this.uptake = uptake;
    this.cartridge = cartridge;
    this.shooter = shooter;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(beltwrap, uptake, cartridge, shooter, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setNeutralMode(NeutralModeValue.Coast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    beltwrap.runVolts(12.0);
    uptake.runVolts(12.0);
    cartridge.runVolts(12.0);
    shooter.setVoltage(5., 5.);
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
