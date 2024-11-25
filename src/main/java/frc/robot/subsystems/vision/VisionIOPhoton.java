package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhoton implements VisionIO {
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonCamera camera;
  Transform3d robotToCam;
  PhotonPoseEstimator poseEstimator;
  PhotonPipelineResult result;

  public VisionIOPhoton() {
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    camera = new PhotonCamera("cam-1NGA");
    robotToCam = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    poseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    result = camera.getLatestResult();
    inputs.hasTargets = result.hasTargets();
    inputs.fieldRelativePose = poseEstimator.getReferencePose();
    inputs.pipeline = camera.getPipelineIndex();
    inputs.bestTagID = result.getBestTarget().getFiducialId();
    inputs.bestTagPoseAmbiguity = result.getBestTarget().getPoseAmbiguity();
    inputs.targetPitch = result.getBestTarget().getPitch();
    inputs.timestamp = result.getTimestampSeconds();
  }

  @Override
  public void switchPipeline(int index) {
    camera.setPipelineIndex(index);
  }
}
