// package frc.robot.subsystems.newFlywheel;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.littletonrobotics.junction.Logger;

// public class Flywheel extends SubsystemBase {
//   private final FlywheelIO flyio;
//   private static Flywheel instance;
//   private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

//   public Flywheel(FlywheelIO io) {
//     flyio = io;
//   }

//   public static Flywheel getInstance() {
//     if (instance != null) {
//       return instance;
//     } else {
//       instance = new Flywheel(new FlywheelIOSim());
//       return instance;
//     }
//   }

//   @Override
//   public void periodic() {

//     flyio.updateInputs(inputs);
//     Logger.processInputs("Flywheel", inputs);
//   }

//   public void setVoltage(double volts) {
//     flyio.setVoltage(volts);
//   }

//   public void setRPM(double rpm) {
//     flyio.setRPM(rpm);
//   }

//   public void off() {
//     flyio.off();
//   }

//   public void setPID(double kP, double kI, double kD, double kS, double kV) {
//     flyio.configurePID(kP, kI, kD, kS, kV);
//   }
// }
