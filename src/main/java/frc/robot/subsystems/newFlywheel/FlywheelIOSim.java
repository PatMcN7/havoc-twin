// package frc.robot.subsystems.newFlywheel;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;

// public class FlywheelIOSim implements FlywheelIO {
//   private FlywheelSim flySim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.4);
//   private PIDController PID = new PIDController(0, 0, 0);
//   private double simVolts = .0;
//   private boolean isOpenLoop = true;

//   public void updateInputs(FlywheelIOInputs inputs) {
//     if (!isOpenLoop) {
//       simVolts = 12 * PID.calculate(inputs.rpm);
//       flySim.setInputVoltage(simVolts);
//     }

//     // update.
//     flySim.update(0.02);

//     // pass out results :)
//     inputs.volts = simVolts;
//     inputs.rpm = flySim.getAngularVelocityRPM();
//     inputs.current = flySim.getCurrentDrawAmps();
//   }

//   public void setVoltage(double volts) {
//     this.simVolts = volts;
//     flySim.setInputVoltage(simVolts);
//     isOpenLoop = true;
//   }

//   public void setRPM(double rpm) {
//     PID.setSetpoint(rpm);
//     isOpenLoop = false;
//   }

//   public void off() {
//     setVoltage(0);
//   }

//   public void configurePID(double kP, double kI, double kD, double kS, double kV) {
//     PID.setPID(kP, kI, kD);
//   }
// }
