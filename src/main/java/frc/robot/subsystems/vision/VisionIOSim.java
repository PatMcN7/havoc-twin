package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;


public class VisionIOSim implements VisionIO {
    VisionSystemSim visionSim = new VisionSystemSim("main");
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    SimCameraProperties cameraProp = new SimCameraProperties();
    Transform3d robotToCam;
    PhotonCamera camera = new PhotonCamera("topCam");
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

    public VisionIOSim(){
        visionSim.addAprilTags(tagLayout);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(0.037),Units.inchesToMeters(-18.030),Units.inchesToMeters(7.693)), new Rotation3d(-90,0,180));
        visionSim.addCamera(cameraSim, robotToCam);


    }   

    @Override
    public void updateInputs(VisionIOInputs inputs){
        visionSim.update(RobotState.getPose());
        
    }

    @Override
    public void switchPipeline(int index){

    }
}