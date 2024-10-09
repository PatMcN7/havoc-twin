package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO{
    private SingleJointedArmSim arm = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 0, 0, 0, 0, 0, false, 0);
    private PIDController pid = new PIDController(0, 0, 0);

    @Override
    public void updateInputs(ArmIOInputs inputs){
        inputs.currentAmps = arm.getCurrentDrawAmps();
        inputs.postionDeg = arm.getAngleRads();
        inputs.velocityDegPerSec = arm.getVelocityRadPerSec();
        inputs.temperature = 0.0;
        inputs.voltageOut = pid.calculate(0);
    }
}
