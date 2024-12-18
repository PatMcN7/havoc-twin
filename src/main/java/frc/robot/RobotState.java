package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotState {
  private static Pose2d odomPose = new Pose2d();

  public static void updatePoseOdom(Pose2d pose) {
    odomPose = pose;
  }

  public static void updateVision(Pose2d pose) {}

  public static Pose2d getPose() {
    return odomPose;
  }
}
