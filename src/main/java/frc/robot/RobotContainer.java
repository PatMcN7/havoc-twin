// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.ArmDefault;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.shoot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.beltwrap.Beltwrap;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.uptake.Uptake;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final Drive drive;
  // private final Shooter shooter;
  // private final Cartridge cartridge;
  // private final Uptake uptake;
  // private final Beltwrap beltwrap;
  // private final Arm arm;

  private Drive drive;
  private Shooter shooter = Shooter.getInstance();
  private Cartridge cartridge = Cartridge.getInstance();
  private Uptake uptake = Uptake.getInstance();
  private Beltwrap beltwrap = Beltwrap.getInstance();
  private Arm arm = Arm.getInstance();
  private final CommandXboxController controller = new CommandXboxController(0);
  // private final XboxControllerSim simController = new XboxControllerSim(0);

  // Controller

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // private final LoggedDashboardNumber flywheelSpeedInput =
  // new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        // shooter = new Shooter(new ShooterIOTalonFX());
        // cartridge = new Cartridge(new CartridgeIOSparkMax());
        // uptake = new Uptake(new UptakeIOTalonFX());
        // beltwrap = new Beltwrap(new BeltwrapIOSparkMax());
        // arm = new Arm(new ArmIOTalonFX());

        // drive = new Drive(
        // new GyroIOPigeon2(true),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // // flywheel = new Flywheel(new FlywheelIOSim());
        // drive =
        // new Drive(
        // new GyroIOPigeon2(false),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // shooter = new Shooter(new ShooterIOTalonFX());
        // cartridge = new Cartridge(new CartridgeIOSparkMax());
        // uptake = new Uptake(new UptakeIOTalonFX());
        // beltwrap = new Beltwrap(new BeltwrapIOSparkMax());
        // arm = new Arm(new ArmIOTalonFX());
        break;

      default:
        // // Replayed robot, disable IO implementations
        // drive =
        // new Drive(
        // new GyroIO() {},
        // new ModuleIO() {},
        // new ModuleIO() {},
        // new ModuleIO() {},
        // new ModuleIO() {});
        // // flywheel = new Flywheel(new FlywheelIO() {});
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        // shooter = new Shooter(new ShooterIOTalonFX());
        // cartridge = new Cartridge(new CartridgeIOSparkMax());
        // uptake = new Uptake(new UptakeIOTalonFX());
        // beltwrap = new Beltwrap(new BeltwrapIOSparkMax());
        // arm = new Arm(new ArmIOTalonFX());
        break;
    }

    // Set up at routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routinChooser.addOption("Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistChooser.addOption("Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistChooser.addOption( "Drive SysId (DynaChooser.addOption(

    // Configure the button bi
    autoChooser.addDefaultOption("Test", drive.followPath("Test Path"));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), .3),
            () -> MathUtil.applyDeadband(-controller.getLeftX(), .3),
            () -> MathUtil.applyDeadband(-controller.getRightX(), .3)));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    controller.a().whileTrue(new Intake(uptake, beltwrap, cartridge, arm, 35.));
    controller.y().onTrue(new shoot(shooter, cartridge, 5000, 5500, arm));
    // controller.x().whileTrue(drive.pathfind(new Pose2d(5.0, 5.00, new Rotation2d(0.0))));
    arm.setDefaultCommand(new ArmDefault(arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
