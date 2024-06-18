package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.MAX_ANGULAR_SPEED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveWithTargetingCommand extends Command {

  private static final double DEADBAND = 0.1;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);

  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.6, MAX_ANGULAR_SPEED * 1.2);

  private final ProfiledPIDController thetaController;

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Supplier<Pose2d> poseProvider;
  private final Supplier<Rotation2d> goalRotation;

  public DriveWithTargetingCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> poseProvider,
      Supplier<Rotation2d> goalRotation) {
    this(drive, xSupplier, ySupplier, poseProvider, goalRotation, DEFAULT_OMEGA_CONSTRAINTS);
  }

  public DriveWithTargetingCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> poseProvider,
      Supplier<Rotation2d> goalRotation,
      TrapezoidProfile.Constraints omegaConstraints) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.poseProvider = poseProvider;
    this.goalRotation = goalRotation;

    thetaController = new ProfiledPIDController(10.0, 0.02, 0.0, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);
  }

  public void initialize() {
    resetPIDControllers();
    thetaController.setGoal(goalRotation.get().getRadians());
  }

  public boolean atGoal() {
    return thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
  }

  public void execute() {
    var robotPose = poseProvider.get();

    // Turn towards the target
    thetaController.setGoal(goalRotation.get().getRadians());
    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }
    Logger.recordOutput("thetaGoal", goalRotation.get().getRadians());
    Logger.recordOutput("thetaSP", thetaController.getSetpoint().position);
    Logger.recordOutput("thetaPV", robotPose.getRotation().getRadians());
    Logger.recordOutput("thetaOP", omegaSpeed);
    // Apply deadband to translation inputs
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calculate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field relative speeds & send command
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omegaSpeed,
            robotPose.getRotation()));
  }

  public void end(boolean interrupted) {
    drive.stop();
  }
}
