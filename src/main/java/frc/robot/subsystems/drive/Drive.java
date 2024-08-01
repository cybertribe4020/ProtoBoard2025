// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private Rotation2d rawGyroRotation = new Rotation2d();

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          new Pose2d(),
          VisionConstants.STD_DEVS_ODOMETRY,
          VisionConstants.STD_DEVS_VISION_DEFAULT);

  private final DigitalInput stageSensor = new DigitalInput(4);

  public boolean isUsingVision = true;
  public boolean driveFieldCentric = true;

  private Translation2d speakerPosition;
  private Translation2d vectorFaceSpeaker;
  private Translation2d lobTarget;
  private Translation2d vectorForLob;
  private Translation2d referencePoint = new Translation2d(0, 0);

  public Pose2d currentPathEndpoint;

  private final SysIdRoutine sysId;
  // configuration settings for SysID
  // Be careful how large your ramp and step voltage is along with how long the test will run
  // You need to provide enough space to let the robot run the test
  private final Measure<Velocity<Voltage>> rampRate = Volts.of(1.5).per(Seconds.of(1));
  private final Measure<Voltage> stepVoltage = Volts.of(7);
  private final Measure<Time> timeout = Seconds.of(4);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Create an AutoBuilder for PathPlanner
    // This is a swerve drivetrain, so you need the holonomic configuration
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            DriveConstants.MAX_LINEAR_SPEED,
            DriveConstants.DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
          currentPathEndpoint =
              (activePath.size() > 0) ? activePath.get(activePath.size() - 1) : getPose();
          Logger.recordOutput("currentPathEndpoint", currentPathEndpoint);
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Create infrastructure to run system identification using the WPILib class
    // The swerve module class needs a runCharacterization method to send the voltages
    // from sys ID to the drive motors and to keep the wheels pointed forward
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                rampRate,
                stepVoltage,
                timeout,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    poseEstimator.update(rawGyroRotation, modulePositions);

    Logger.recordOutput("Is Using Vision?", isUsingVision);
    Logger.recordOutput("driveFieldCentric", driveFieldCentric);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic sysID test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic sysID test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  @AutoLogOutput(key = "ChassisSpeeds/Measured")
  private ChassisSpeeds getMeasuredSpeeds() {
    var cSpeeds = kinematics.toChassisSpeeds(getModuleStates());
    Logger.recordOutput(
        "ChassisSpeeds/LinearVelocity",
        Math.hypot(cSpeeds.vxMetersPerSecond, cSpeeds.vyMetersPerSecond));
    return cSpeeds;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * Returns the current odometry pose. This pose is fused with estimates from vision if vision is
   * being used
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    if (isUsingVision) poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> estStdDevs) {
    if (isUsingVision) poseEstimator.addVisionMeasurement(visionPose, timestamp, estStdDevs);
    // SmartDashboard.putNumber("Vision pose X:", visionPose.getX());
    // SmartDashboard.putNumber("Vision pose Y:", visionPose.getY());
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  // What is the distance and angle to the speaker from the current robot pose?
  public Translation2d getVectorFaceSpeaker() {
    speakerPosition = FieldConstants.getSpeakerPosition();
    vectorFaceSpeaker = speakerPosition.minus(getPose().getTranslation());
    // Logger.recordOutput("vectorFaceSpeaker", vectorFaceSpeaker);
    return vectorFaceSpeaker;
  }

  // What is the distance and angle to the lob target from the current robot pose
  public Translation2d getVectorForLob() {
    if (FieldConstants.isBlue()) {
      lobTarget = new Translation2d(1.9, 6.5);
    } else {
      lobTarget = new Translation2d(VisionConstants.FIELD_LENGTH - 1.9, 6.5);
    }
    vectorForLob = lobTarget.minus(getPose().getTranslation());
    Logger.recordOutput("distanceForLob", vectorForLob.getNorm());
    return vectorForLob;
  }

  // extract just the distance to the speaker from the vector relative to the robot position
  // this method adjusts the distance to be equivalent to the back camera lens
  // rather than the robot center, since the speaker shooting calibration
  // that uses this value uses the distance from the back camera lens
  public double getDistToSpeaker() {
    return getVectorFaceSpeaker().getNorm() - 0.28;
  }

  // extract just the distance to the speaker from the endpoint of the current
  // PathPlanner path segment
  // this method adjusts the distance to be equivalent to the back camera lens
  // rather than the robot center, since the speaker shooting calibration
  // that uses this value uses the distance from the back camera lens
  public double getPathEndToSpeaker() {
    speakerPosition = FieldConstants.getSpeakerPosition();
    vectorFaceSpeaker = speakerPosition.minus(currentPathEndpoint.getTranslation());
    return vectorFaceSpeaker.getNorm() - 0.28;
  }

  // Shoot wider than the direct angle to the speaker Apriltag to give more clearance for the Note
  public double getBiasedShootingRot(double directRotation) {
    if (FieldConstants.isBlue()) {
      return directRotation * ShooterConstants.ANGLE_BIAS_MULTIPLIER;
    } else {
      var flippedRotation = MathUtil.angleModulus(directRotation + Math.PI);
      return MathUtil.angleModulus(
          (flippedRotation * ShooterConstants.ANGLE_BIAS_MULTIPLIER) - Math.PI);
    }
  }

  // The stage sensor reads True with clear space overhead and False when it is under the stage
  @AutoLogOutput
  public boolean underStage() {
    return !stageSensor.get();
  }

  public double getDistFromPointM() {
    var distance = getPose().getTranslation().minus(referencePoint).getNorm();
    Logger.recordOutput("Drive/distFromRef", distance);
    return distance;
  }

  public void setReferencePoint() {
    referencePoint = getPose().getTranslation();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED;
  }

  /** Returns an array of swerve module translations relative to the center of the robot. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
    };
  }

  // Get an array of individual module poses
  // This can be used to visualize the robot on the field, showing each module
  // and the angle of each wheel
  public Pose2d[] getModulePoses() {
    Pose2d[] modulePoses = new Pose2d[modules.length];
    for (int i = 0; i < modules.length; i++) {
      var module = modules[i];
      modulePoses[i] =
          getPose().transformBy(new Transform2d(getModuleTranslations()[i], module.getAngle()));
    }
    return modulePoses;
  }
}
