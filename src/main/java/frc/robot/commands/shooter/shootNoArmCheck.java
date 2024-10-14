package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.shooter.Shooter;

public class shootNoArmCheck extends Command {

  Shooter shooter;
  Cartridge cartridge;
  double leftVelocityRPM;
  double rightVelocityRPM;
  boolean flag;
  Arm arm;
  /** Creates a new Shooter. */
  public shootNoArmCheck(
      Shooter shooter, Cartridge cartridge, double leftVelocityRPM, double rightVelocityRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.cartridge = cartridge;
    this.leftVelocityRPM = leftVelocityRPM;
    this.rightVelocityRPM = rightVelocityRPM;
    flag = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setRPM(leftVelocityRPM, rightVelocityRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setRPM(leftVelocityRPM, rightVelocityRPM);

    if (shooter.getLeftAtSetpoint() && shooter.getRightAtSetpoint()) {
      cartridge.runVolts(12.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cartridge.runVolts(0.0);
    shooter.setVoltage(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!cartridge.getFirstBeam() && !cartridge.getSecondBeam()) {
      return true;
    }
    return false;
  }
}
