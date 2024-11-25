package frc.lib.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim sim;

  public ElevatorIOSim() {
    sim = new ElevatorSim(null, null, 0, 0, false, 0);
    sim.getPositionMeters();
  }
}
