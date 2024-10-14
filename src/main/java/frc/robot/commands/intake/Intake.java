// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.beltwrap.Beltwrap;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.uptake.Uptake;

public class Intake extends Command {
  private Uptake uptake;
  private Beltwrap beltwrap;
  private Cartridge cartridge;
  private Arm arm;
  private double armPos;
  /** Creates a new Intake. */
  public Intake(Uptake uptake, Beltwrap beltwrap, Cartridge cartridge, Arm arm, double armPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.uptake = uptake;
    this.beltwrap = beltwrap;
    this.cartridge = cartridge;
    this.arm = arm;
    this.armPos = armPos;
    addRequirements(uptake, beltwrap, cartridge, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // arm.setPosition(35.0);
    arm.setPosition(armPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setPosition(armPos);

    if (arm.atPosition() && !cartridge.getFirstBeam()) {
      uptake.runVolts(12.0);
      beltwrap.runVolts(12.0);
      cartridge.runVolts(12.0);
    }
    if (cartridge.getFirstBeam()) {
      uptake.runVolts(4.);
      cartridge.runVolts(3.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cartridge.runVolts(0.0);
    uptake.runVolts(0.0);
    beltwrap.runVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (cartridge.getSecondBeam()) {
      return true;
    } else {
      return false;
    }
  }
}
