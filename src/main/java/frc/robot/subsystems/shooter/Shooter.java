package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 1.0);
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  // private final SysIdRoutine sysId;

  public Shooter(ShooterIO io) {
    switch (Constants.currentMode) {
      case REAL:
        io.configureGains(0., 0., 0., 0., 0.);
    }

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setRightVoltage(double volts) {
    io.setRightVoltage(volts);
  }

  public void setLeftVoltage(double volts) {
    io.setLeftVoltage(volts);
  }

  public void setRightRPM(double RPM) {
    io.setRightRPM(RPM);
  }

  public void setLeftRPM(double RPM) {
    io.setLeftRPM(RPM);
  }
}
