package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public Pose3d fieldRelativePose;
    public double timestamp;
    public boolean hasTargets;
    public double targetPitch;
    public int pipeline;
    public int bestTagID;
    public double bestTagPoseAmbiguity;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void switchPipeline(int newPipeline) {}
}
