package frc.robot.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Simulate a winched rope used as a climber. In the extend mode, the rope is tensioned with
 * tensionEquivMassKg. In the climbing mode, the rope is tensioned with robotMassKg. Run two
 * ElevatorSims in parallel with these two different masses. Update the model that is active and
 * sync the states for the inactive model.
 */
public class ClimberSim extends ElevatorSim {
  private final ElevatorSim climbingSim;
  private boolean isClimbing = false;

  public ClimberSim(
      DCMotor motor,
      double reduction,
      double tensionEquivMassKg,
      double robotMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double startHeightMeters) {

    // this is the rope with spring tension
    // for now, turn off gravity so rope stays stationary in open-loop
    // tiny tension force is like running the motor unloaded
    super(
        motor,
        reduction,
        tensionEquivMassKg,
        drumRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        false,
        startHeightMeters);

    // this is the rope lifting the robot
    this.climbingSim =
        new ElevatorSim(
            motor,
            reduction,
            robotMassKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            true,
            startHeightMeters);
  }

  private void copyTensionedRopeStateToClimberSim() {
    climbingSim.setState(super.getPositionMeters(), super.getVelocityMetersPerSecond());
  }

  private void copyClimberStateToTensionedRopeSim() {
    super.setState(climbingSim.getPositionMeters(), climbingSim.getVelocityMetersPerSecond());
  }

  public void climb() {
    isClimbing = true;
  }

  public void extend() {
    isClimbing = false;
  }

  public void setParallelStates(double positionMeters, double velocityMetersPerSecond) {
    climbingSim.setState(positionMeters, velocityMetersPerSecond);
    super.setState(positionMeters, velocityMetersPerSecond);
  }

  public boolean inClimbingMode() {
    return isClimbing;
  }

  @Override
  public void setInputVoltage(double volts) {
    climbingSim.setInputVoltage(volts);
    super.setInputVoltage(volts);
  }

  @Override
  public void update(double dtSeconds) {
    if (isClimbing) {
      climbingSim.update(dtSeconds);
      copyClimberStateToTensionedRopeSim();
      // after copying state, must update inactive model for the states to "take"
      // use a tiny interval so copied values are essentially unchanged
      super.update(0.000001);
    } else {
      super.update(dtSeconds);
      copyTensionedRopeStateToClimberSim();
      // after copying state, must update inactive model for the states to "take"
      // use a tiny interval so copied values are essentially unchanged
      climbingSim.update(0.000001);
    }
  }
}
