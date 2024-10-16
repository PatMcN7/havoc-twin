package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public double[] fieldRelativePose;
    public double timestamp;
    public boolean hasTargets;
    public double targetPitch;
    public int pipeline;
    public int tagID;
    public double poseAmbiguity;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void switchPipeline(int newPipeline) {}
}
