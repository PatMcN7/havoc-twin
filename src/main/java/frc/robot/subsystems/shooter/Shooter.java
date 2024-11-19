package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 1.0);
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private static Shooter instance;
  // private final SysIdRoutine sysId;

  public Shooter(ShooterIO io) {
    switch (Constants.currentMode) {
      case REAL:
        io.configureGains(.28, 0.12, 0.12, 0., 0.);
      case SIM:
        io.configureGains(.28, 0.12, 0.12, 0., 0.);
    }
    Pose2d pose = new Pose2d();
    this.io = io;
    // sysId =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //             null,
    //             null,
    //             null,
    //             (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
    //         new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));

  }

  public static Shooter getInstance() {
    if (instance == null) {
      if (Constants.currentMode.equals(Constants.Mode.REAL)) {
        return instance = new Shooter(new ShooterIOTalonFX());
      } else if (Constants.currentMode.equals(Constants.Mode.SIM)) {
        System.out.println("Shooter works");

        return instance = new Shooter(new ShooterIOSim());
      } else {
        return instance;
      }
    } else {
      return instance;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
    ;
  }

  public void setRPM(double leftRPM, double rightRPM) {
    io.setRPM(leftRPM, rightRPM);
  }

  public boolean getLeftAtSetpoint() {
    return inputs.leftAtVelocity;
  }

  public boolean getRightAtSetpoint() {
    return inputs.rightAtVelocity;
  }
}
