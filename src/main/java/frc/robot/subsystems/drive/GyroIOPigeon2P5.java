package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
// Phoenix 5 is deprecated, so suppress warnings so code is readable in the editor
@SuppressWarnings({"deprecation", "removal"})
public class GyroIOPigeon2P5 implements GyroIO {
  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(43);

  public GyroIOPigeon2P5() {
    pigeon.setYaw(0.0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true; // not sure how to check connectedness in Phoenix5
    inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw());
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(
            pigeon.getRate()); // getRate is positive for clockwise rotations if Pigeon case top
    // points up
  }
}
