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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.arm1.Arm1;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveWithTargetingCommand extends Command {

  private static final double DEADBAND = 0.1;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);
  private static final double ANGLE_BIAS_MULTIPLIER = 1.1;

  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.6, MAX_ANGULAR_SPEED * 1.2);

  private final Rotation2d facingBackwards = new Rotation2d(Math.PI);

  private final ProfiledPIDController thetaController;

  private double distToSpeaker;

  private final Drive drive;
  private final Shooter shooter;
  private final Arm1 arm1;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Supplier<Pose2d> poseProvider;
  private final Supplier<Translation2d> goalVector;

  public DriveWithTargetingCommand(
      Drive drive,
      Shooter shooter,
      Arm1 arm1,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> poseProvider,
      Supplier<Translation2d> goalVector) {
    this(
        drive,
        shooter,
        arm1,
        xSupplier,
        ySupplier,
        poseProvider,
        goalVector,
        DEFAULT_OMEGA_CONSTRAINTS);
  }

  public DriveWithTargetingCommand(
      Drive drive,
      Shooter shooter,
      Arm1 arm1,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> poseProvider,
      Supplier<Translation2d> goalVector,
      TrapezoidProfile.Constraints omegaConstraints) {
    this.drive = drive;
    this.shooter = shooter;
    this.arm1 = arm1;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.poseProvider = poseProvider;
    this.goalVector = goalVector;

    thetaController = new ProfiledPIDController(5.0, 0.02, 0.0, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);
  }

  public void initialize() {
    resetPIDControllers();
    thetaController.setGoal(
        biasedRotation(goalVector.get().getAngle().plus(facingBackwards).getRadians()));
  }

  public void execute() {
    var robotPose = poseProvider.get();

    // Turn towards the target
    thetaController.setGoal(
        biasedRotation(goalVector.get().getAngle().plus(facingBackwards).getRadians()));
    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }
    Logger.recordOutput(
        "thetaGoal", goalVector.get().getAngle().plus(facingBackwards).getRadians());
    Logger.recordOutput("thetaSP", thetaController.getSetpoint().position);
    Logger.recordOutput("thetaPV", robotPose.getRotation().getRadians());
    Logger.recordOutput("thetaOP", omegaSpeed);

    // set the shooter speed and angle
    // get distance to center of robot and subtract 0.28 meters of offset to camera lens
    distToSpeaker = goalVector.get().getNorm() - 0.28;
    shooter.runVolts(12.0 * ShooterConstants.SPEED_MAP.get(distToSpeaker));
    arm1.setGoalDeg(ShooterConstants.ANGLE_MAP.get(distToSpeaker));

    Logger.recordOutput("feedToSpeaker", Units.metersToFeet(distToSpeaker));
    Logger.recordOutput("speed map value", ShooterConstants.SPEED_MAP.get(distToSpeaker));
    Logger.recordOutput("angle map value", ShooterConstants.ANGLE_MAP.get(distToSpeaker));

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
            FieldConstants.shouldFlip()
                ? robotPose.getRotation().plus(new Rotation2d(Math.PI))
                : robotPose.getRotation()));
  }

  public void end(boolean interrupted) {
    drive.stop();
    shooter.stop();
    arm1.setGoalDeg(ArmConstants.ARM_STOW_ANGLE_DEG);
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
  }

  public boolean atGoal() {
    return thetaController.atGoal();
  }

  private double biasedRotation(double directRotation) {
    if (FieldConstants.isBlue()) {
      return directRotation * ANGLE_BIAS_MULTIPLIER;
    } else {
      var flippedRotation = MathUtil.angleModulus(directRotation + Math.PI);
      return MathUtil.angleModulus((flippedRotation * ANGLE_BIAS_MULTIPLIER) - Math.PI);
    }
  }
}
