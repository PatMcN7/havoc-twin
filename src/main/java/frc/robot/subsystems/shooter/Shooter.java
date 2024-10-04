package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTunableNumber;

public class Shooter {
  LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 1.0);

  public Shooter() {}
}
