package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ArmVisualizer {
  private final Mechanism2d mechanism;
  private final MechanismLigament2d arm;
  private final String name;

  public ArmVisualizer(String name, Color color) {
    this.name = name;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    MechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
    arm = new MechanismLigament2d("arm", 0.355, 5.0, 6, new Color8Bit(color));
    root.append(arm);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double angleDeg) {
    // Log Mechanism2d
    arm.setAngle(Rotation2d.fromDegrees(angleDeg));
    Logger.recordOutput("Arm/Mechanism2d/" + name, mechanism);

    // Log 3D poses
    Pose3d pivot =
        new Pose3d(-0.03, 0.0, .1, new Rotation3d(0.0, Units.degreesToRadians(angleDeg), 0.0));
    Logger.recordOutput("Arm/Mechanism3d/" + name, pivot);
  }
}
