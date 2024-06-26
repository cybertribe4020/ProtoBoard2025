package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.MAX_ANGULAR_SPEED;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class TurnToAngleCommand extends ProfiledPIDCommand {

  public TurnToAngleCommand(Drive drive, DoubleSupplier targetAngleRad) {
    super(
        new ProfiledPIDController(
            5.0,
            0.02,
            0.0,
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.6, MAX_ANGULAR_SPEED * 1.2)),
        () -> drive.getRotation().getRadians(),
        targetAngleRad,
        (output, setpoint) -> {
          drive.runVelocity(new ChassisSpeeds(0.0, 0.0, output));
          // Logger.recordOutput("Turn/thetaOP", output);
          // Logger.recordOutput("Turn/thetaSP", setpoint.position);
        },
        drive);

    getController().enableContinuousInput(-Math.PI, Math.PI);
    getController().setTolerance(Units.degreesToRadians(1.0));
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
