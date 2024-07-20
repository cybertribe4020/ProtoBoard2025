package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.THETA_kD;
import static frc.robot.Constants.AutoConstants.THETA_kI;
import static frc.robot.Constants.AutoConstants.THETA_kP;
import static frc.robot.Constants.AutoConstants.X_kD;
import static frc.robot.Constants.AutoConstants.X_kI;
import static frc.robot.Constants.AutoConstants.X_kP;
import static frc.robot.Constants.AutoConstants.Y_kD;
import static frc.robot.Constants.AutoConstants.Y_kI;
import static frc.robot.Constants.AutoConstants.Y_kP;
import static frc.robot.Constants.DriveConstants.MAX_ANGULAR_SPEED;
import static frc.robot.Constants.DriveConstants.MAX_LINEAR_SPEED;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {

  private static final double TRANSLATION_TOLERANCE = 0.02;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  // Default profile constraints are used if constraints are not passed to the method
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_LINEAR_SPEED * 0.5, MAX_LINEAR_SPEED);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.4, MAX_ANGULAR_SPEED);

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final Drive drive;
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;
  private final boolean useAllianceColor;

  public DriveToPoseCommand(
      Drive drive, Supplier<Pose2d> poseProvider, Pose2d goalPose, boolean useAllianceColor) {
    this(
        drive,
        poseProvider,
        goalPose,
        DEFAULT_XY_CONSTRAINTS,
        DEFAULT_OMEGA_CONSTRAINTS,
        useAllianceColor);
  }

  public DriveToPoseCommand(
      Drive drive,
      Supplier<Pose2d> poseProvider,
      Pose2d goalPose,
      TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints,
      boolean useAllianceColor) {
    this.drive = drive;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.useAllianceColor = useAllianceColor;

    // Controllers are currently using the autonomous tuning for the profiled PID
    // If different tuning is needed for teleop, a different set of constants could be imported
    xController = new ProfiledPIDController(X_kP, X_kI, X_kD, xyConstraints);
    yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, xyConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);
  }

  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;
    
    // If red alliance and we are asking to flip a blue (default) pose
    // to red when on the red alliance, then flip the pose
    if (useAllianceColor && FieldConstants.shouldFlip()) {
      Translation2d transformedTranslation =
          new Translation2d(FIELD_LENGTH - pose.getX(), pose.getY());
      Rotation2d transformedHeading =
          pose.getRotation().times(-1).plus(Rotation2d.fromDegrees(180));
      pose = new Pose2d(transformedTranslation, transformedHeading);
    }

    // Set the goal for each of the controllers to the appropriate
    // component of the goal pose
    thetaController.setGoal(pose.getRotation().getRadians());
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());
  }

  public void execute() {
    var robotPose = poseProvider.get();

    // Drive to each component of the goal pose simultaneously
    // x translation, y translation, and rotation
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }
    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }
    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }
    // Pass the controller speed targets to the drive
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

  public boolean isFinished() {
    return atGoal();
  }

  public void end(boolean interrupted) {
    drive.stop();
  }

  // The move is completely finished when all three controllers reach goal
  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  // initializaton
  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }
}
