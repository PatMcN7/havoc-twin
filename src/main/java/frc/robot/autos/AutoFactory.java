package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class AutoFactory {
    private final DriverStation.Alliance alliance;
    private final RobotContainer robotContainer; 
    private final Drive drive; // Need to change the drive class to have a getInstance so that we don't create 30 drive classes

    AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer, Drive drive){
        this.alliance = alliance;
        this.robotContainer = robotContainer;
        this.drive = drive;
    }

    Command createOneMeterForward(){
        SequentialCommandGroup c = new SequentialCommandGroup();
        return c;
    }

    
}
