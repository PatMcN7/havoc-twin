package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim arm =
      new SingleJointedArmSim(DCMotor.getKrakenX60(1), 1, 1., 1., 0, 2, false, 0);
  private PIDController pid = new PIDController(0, 0, 0);
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = MathUtil.clamp(pid.calculate(arm.getAngleRads()), -12.0, 12.0);
      arm.setInputVoltage(appliedVolts);
    }

    arm.update(.02);

    inputs.currentAmps = arm.getCurrentDrawAmps();
    inputs.postionDeg = arm.getAngleRads();
    inputs.velocityDegPerSec = arm.getVelocityRadPerSec();
    inputs.temperature = 0.0;
    inputs.voltageOut = appliedVolts;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    arm.setInput(volts);
  }

  @Override
  public void setPosition(double position) {
    closedLoop = true;
    pid.setSetpoint(position);
  }

  @Override
  public void configurePID(double kS, double kV, double kA, double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }
}
