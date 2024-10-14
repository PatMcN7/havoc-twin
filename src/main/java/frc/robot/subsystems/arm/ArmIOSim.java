package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private double setpointDeg;
  private SingleJointedArmSim arm =
      new SingleJointedArmSim(DCMotor.getKrakenX60(1), 47.6, 1., 1., 0, 2, true, 0);
  private PIDController pid = new PIDController(.5, 0., 0.01);
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private DIOSim limitSwitch = new DIOSim(9);

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(Units.radiansToDegrees(arm.getAngleRads())), -12.0, 12.0);
      arm.setInputVoltage(appliedVolts);
    }
    if (arm.getAngleRads() == Units.degreesToRadians(0)) {
      limitSwitch.setValue(true);
    } else {
      limitSwitch.setValue(false);
    }
    arm.update(.02);

    inputs.currentAmps = arm.getCurrentDrawAmps();
    inputs.postionDeg = Units.radiansToDegrees(arm.getAngleRads());
    inputs.velocityDegPerSec = Units.radiansToDegrees(arm.getVelocityRadPerSec());
    inputs.temperature = 0.0;
    inputs.voltageOut = appliedVolts;
    inputs.setpointDeg = setpointDeg;
    inputs.atZero = limitSwitch.getValue();
    inputs.currentAmps = arm.getCurrentDrawAmps();
    inputs.atPosition =
        MathUtil.isNear(0.0, setpointDeg - Units.radiansToDegrees(arm.getAngleRads()), 3.0);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    arm.setInput(volts);
  }

  @Override
  public void setPosition(double position) {
    setpointDeg = position;
    closedLoop = true;
    pid.setSetpoint(position);
  }

  @Override
  public void configurePID(double kS, double kV, double kA, double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  @Override
  public void zeroArm() {
    arm.setState(0, 0);
  }
}
