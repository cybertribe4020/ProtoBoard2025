package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.THETA_kD;
import static frc.robot.Constants.AutoConstants.THETA_kI;
import static frc.robot.Constants.AutoConstants.THETA_kP;
import static frc.robot.Constants.DriveConstants.MAX_ANGULAR_SPEED;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// This command uses a single profiled PID controller to rotate the robot
// in the current position to face a target angle
// The constraints for velocity and acceleration of rotation as well as the
// tolerance for finishing the rotation are static constants
// The command could be overloaded to make these settable parameters
public class TurnToAngleCommand extends ProfiledPIDCommand {

  public TurnToAngleCommand(Drive drive, DoubleSupplier targetAngleRad) {
    super(
        // Theta controller uses autonomous tuning
        // Can import some different tuning if desired
        // There is also currently just default trapezoid profile constraints
        // Could add a separate constructor with passed-in constraints if desired
        new ProfiledPIDController(
            THETA_kP,
            THETA_kI,
            THETA_kD,
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.6, MAX_ANGULAR_SPEED * 1.2)),
        () -> drive.getRotation().getRadians(),
        targetAngleRad,
        (output, setpoint) -> {
          drive.runVelocity(new ChassisSpeeds(0.0, 0.0, output));
          Logger.recordOutput("TurnToAngle/thetaOP", output);
          Logger.recordOutput("TurnToAngle/thetaSP", setpoint.position);
        },
        drive);

    getController().enableContinuousInput(-Math.PI, Math.PI);
    getController().setTolerance(Units.degreesToRadians(1.0));
  }

  @Override
  public boolean isFinished() {
    Logger.recordOutput("TurnToAngle/atGoal", getController().atGoal());
    return getController().atGoal();
  }
}
