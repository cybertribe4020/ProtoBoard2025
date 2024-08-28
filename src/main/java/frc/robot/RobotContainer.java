package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants.StageLocation;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.DriveWithTargetingCommand;
import frc.robot.commands.SmartConveyCommand;
import frc.robot.commands.SmartIntakeCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.arm1.Arm1;
import frc.robot.subsystems.arm1.Arm1IO;
import frc.robot.subsystems.arm1.Arm1IOSim;
import frc.robot.subsystems.arm1.Arm1IOSparkFlex;
import frc.robot.subsystems.convey.Convey;
import frc.robot.subsystems.convey.ConveyIO;
import frc.robot.subsystems.convey.ConveyIOSim;
import frc.robot.subsystems.convey.ConveyIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2P5;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIO4020P5;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.winch.Winch;
import frc.robot.subsystems.winch.WinchIO;
import frc.robot.subsystems.winch.WinchIOSim;
import frc.robot.subsystems.winch.WinchIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// Define the robot here:
// - All subsystems
// - Controllers
// - Binding controller button events to commands
// - Triggers (binding arbitrary events to commands)
// - Named commands for PathPlanner
// - An autonomous routine chooser for the driver dashboard

// Commands for buttons can be created within the button mapping if they
// are reasonably simple and easy to understand

// More complex commands that involve multiple subsytems can be built here
// using "command factory" methods since all the subsystems are already available

// It is also acceptble to build complex commands as classes elsewhere in the codebase
// but if it can be done here with command factory methods, it will take a lot less
// boilerplate code to make it happen

@SuppressWarnings({"unused"})
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Intake intake;
  private final Convey convey;
  private final Arm1 arm;
  private final Winch winchLeft;
  private final Winch winchRight;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2P5(),
                new ModuleIO4020P5(0),
                new ModuleIO4020P5(1),
                new ModuleIO4020P5(2),
                new ModuleIO4020P5(3));
        vision = new Vision(drive);
        shooter = new Shooter(new ShooterIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        convey = new Convey(new ConveyIOSparkMax());
        arm = new Arm1(new Arm1IOSparkFlex());
        winchLeft = new Winch(new WinchIOSparkMax(30, false), "Left");
        winchRight = new Winch(new WinchIOSparkMax(31, true), "Right");
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision = new Vision(drive);
        shooter = new Shooter(new ShooterIOSim());
        intake = new Intake(new IntakeIOSim());
        convey = new Convey(new ConveyIOSim());
        arm = new Arm1(new Arm1IOSim());
        winchLeft = new Winch(new WinchIOSim(), "Left");
        winchRight = new Winch(new WinchIOSim(), "Right");
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive);
        shooter = new Shooter(new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        convey = new Convey(new ConveyIO() {});
        arm = new Arm1(new Arm1IO() {});
        winchLeft = new Winch(new WinchIO() {}, "Left");
        winchRight = new Winch(new WinchIO() {}, "Right");
        break;
    }

    // Register commands that will be used in PathPlanner
    // If a command is not registered, it is not available for use
    NamedCommands.registerCommand(
        "smartIntake",
        new SmartIntakeCommand(intake, () -> arm.armIsDown(), () -> convey.noteIsLoaded()));
    NamedCommands.registerCommand("runConvey", convey.loadCommand());
    NamedCommands.registerCommand("prepareShooter", prepareShooterCommand());
    NamedCommands.registerCommand("turnToSpeaker", turnToSpeakerCommand());
    NamedCommands.registerCommand("aimAtSpeaker", aimAtSpeakerCommand());
    NamedCommands.registerCommand("shoot", shootCommand());
    NamedCommands.registerCommand("lowerArm", arm.armToLoadCommand());
    NamedCommands.registerCommand("stopShooter", new InstantCommand(() -> shooter.stop(), shooter));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> intake.stop(), intake));
    NamedCommands.registerCommand("stopConvey", new InstantCommand(() -> convey.stop(), convey));

    // PathPlanner will build a chooser for the auto routines that have been created in the
    // application
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // If you want to use the sys ID capability in WPILib to compute PID tuning for the drive base
    // one way is to create auto routines to run each of the four necessary tests
    // The tests are defined in the subsystem to be ID'd
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Left stick drive translation front-back, left-right
    // Right stick chassis rotation on x-axis of stick, left CCW, right CW
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Left stick press-down button
    // Drive robot centric
    controller
        .leftStick()
        .onTrue(Commands.runOnce(() -> drive.driveFieldCentric = false).ignoringDisable(true));

    // Right stick press-down button
    // Drive field centric
    controller
        .rightStick()
        .onTrue(Commands.runOnce(() -> drive.driveFieldCentric = true).ignoringDisable(true));

    // POV - drive translation slowly
    // Go in the direction of the POV
    // Can move field or robot-centric depending on the current selected drive mode
    controller
        .povLeft()
        .whileTrue(
            DriveCommands.driveByValues(drive, 0.0, 0.08, 0.0, () -> drive.driveFieldCentric));
    controller
        .povDown()
        .whileTrue(
            DriveCommands.driveByValues(drive, -0.08, 0.0, 0.0, () -> drive.driveFieldCentric));
    controller
        .povUp()
        .whileTrue(
            DriveCommands.driveByValues(drive, 0.08, 0.0, 0.0, () -> drive.driveFieldCentric));
    controller
        .povRight()
        .whileTrue(
            DriveCommands.driveByValues(drive, 0.0, -0.08, 0.0, () -> drive.driveFieldCentric));

    // X button
    // reserved for manual shooting into the trap
    // command is part of the climbing sequence

    // Y Button
    // Drive to a staging location in front of the Amp
    // Will not avoid any obstacles in the way!!
    // Simultaneously raise the arm and run the shooter as needed for the Amp
    controller
        .y()
        .whileTrue(
            new DriveToPoseCommand(
                    drive,
                    drive::getPose, // could also use () -> drive.getPose()
                    () -> new Pose2d(1.82, 7.49, Rotation2d.fromDegrees(270.0)),
                    true)
                .alongWith(prepareForAmpCommand()));

    // A button
    // Drive to the climb start location in front of the closest stage face
    // When done driving, put robot in robot-centric drive mode to enable sideways adjustment
    controller
        .a()
        .whileTrue(
            new DriveToPoseCommand(
                    drive,
                    drive::getPose, // could also use () -> drive.getPose()
                    () -> getNearestStagePose(!FieldConstants.isBlue(), 44.0),
                    false)
                .andThen(new InstantCommand(() -> drive.driveFieldCentric = false)));

    // A button
    // Drive to the climb start location in front of stage left
    // Will avoid stage trusses with PathPlanner
    // Note that pathfindToPoseFlipped could not be used becuase stage left is
    // rotationally symmetric about the field center, not "flipped"
    // about the center line
    // When done driving, put robot in robot-centric drive mode to enable sideways adjustment
    /*
    controller
        .a()
        .whileTrue(
            Commands.either(
                    AutoBuilder.pathfindToPose(
                        getStagePose(FieldConstants.StageLocation.LEFT, false, 44.0),
                        new PathConstraints(
                            3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                        0.0,
                        0.0),
                    AutoBuilder.pathfindToPose(
                        getStagePose(FieldConstants.StageLocation.LEFT, true, 44.0),
                        new PathConstraints(
                            3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                        0.0,
                        0.0),
                    FieldConstants::isBlue)
                .andThen(
                    new DriveToPoseCommand(
                        drive,
                        drive::getPose, // could also use () -> drive.getPose()
                        () ->
                            getStagePose(
                                FieldConstants.StageLocation.LEFT, !FieldConstants.isBlue(), 44.0),
                        false))
                .andThen(new InstantCommand(() -> drive.driveFieldCentric = false)));
    */

    // B button
    // Climb sequence
    // Press and release the B button to run the sequence
    // If there is a problem, press and release the B button again to cancel
    controller.b().toggleOnTrue(ClimbCommand());

    // Left Bumper
    // Continuous chassis rotation to face lob target but allow joystick chassis translation
    // Set shooter speed and angle continouously based on range to lob target
    controller
        .leftBumper()
        .whileTrue(
            new DriveWithTargetingCommand(
                drive,
                shooter,
                arm,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                drive::getPose,
                drive::getVectorForLob,
                true));

    // Right Bumper
    // Run the intake and conveyor
    // For now, this is set to toggle to be able to turn these motors off while testing
    controller
        .rightBumper()
        .toggleOnTrue(
            Commands.parallel(
                new SmartIntakeCommand(intake, arm::armIsDown, convey::noteIsLoaded),
                new SmartConveyCommand(
                    convey,
                    arm::armIsDown,
                    convey::noteIsLoaded,
                    shooter::shooterIsRunning,
                    () -> controller.getRightTriggerAxis() > 0.5)));

    // Left Trigger
    // Continuous chassis rotation to face speaker but allow joystick chassis translation
    // Set shooter speed and angle continouously based on range to speaker
    controller
        .leftTrigger(0.5)
        .whileTrue(
            new DriveWithTargetingCommand(
                drive,
                shooter,
                arm,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                drive::getPose,
                drive::getVectorFaceSpeaker,
                false));

    // Right Trigger
    // Shoot a Note into the Speaker or Amp by running the conveyor
    // Shooter must be running to shoot
    // If the arm angle is nearly vertical, we are shooting into the Amp
    // In this case, run a more complicated sequence for the Amp that
    // pushes the arm more forward while backing the base away, and then lowers the arm
    // and turns off the shooter
    controller
        .rightTrigger(0.5)
        .and(() -> shooter.shooterIsRunning())
        .onTrue(
            new ConditionalCommand(shootCommand(), depositAmpCommand(), () -> arm.armIsNotAmped()));

    // Start button
    // Drive a path with obstacle avoidance to climb start location in front of red stage left
    // Note that the pathplanner only gets within a "navgrid" resolution of the target pose
    // Need to finish with a final driveToPose to fully get to the target pose
    controller
        .start()
        .whileTrue(
            AutoBuilder.pathfindToPose(
                    new Pose2d(12.6, 2.4, Rotation2d.fromDegrees(300.0)),
                    new PathConstraints(
                        3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                    0.0,
                    0.0)
                .andThen(
                    new DriveToPoseCommand(
                        drive,
                        drive::getPose, // could also use () -> drive.getPose()
                        () -> new Pose2d(12.6, 2.4, Rotation2d.fromDegrees(300.0)),
                        false)));

    // Back button
    // Toggle the use of vision for pose estimation
    controller
        .back()
        .onTrue(
            Commands.runOnce(() -> drive.isUsingVision = !drive.isUsingVision)
                .ignoringDisable(true));

    // X button
    // Stop the drivetrain and turn the wheels to an X position
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    /* // X button
    // Test running the winches to the 0 inch point
    // fully extended to catch the chain
    controller
        .x()
        .onTrue(
            Commands.parallel(
                    // Tuning for extension has less load and moves more slowly
                    new InstantCommand(() -> winchLeft.winchTuning(false), winchLeft),
                    new InstantCommand(() -> winchRight.winchTuning(false), winchRight))
                .andThen(
                    Commands.parallel(
                        new InstantCommand(() -> winchLeft.setGoalInch(0.0), winchLeft),
                        new InstantCommand(() -> winchRight.setGoalInch(0.0), winchRight)))); */

    /* // A button
    // Test running the winches to the starting point (8.34 inches)
    controller
        .a()
        .onTrue(
            Commands.parallel(
                    // Run at the slow extension rate for testing
                    new InstantCommand(() -> winchLeft.winchTuning(false), winchLeft),
                    new InstantCommand(() -> winchRight.winchTuning(false), winchRight))
                .andThen(
                    Commands.parallel(
                        new InstantCommand(
                            () -> winchLeft.setGoalInch(WinchConstants.WINCH_START_HEIGHT_IN),
                            winchLeft),
                        new InstantCommand(
                            () -> winchRight.setGoalInch(WinchConstants.WINCH_START_HEIGHT_IN),
                            winchRight)))); */

    /* // B button
    // Test running the winches to the 25.5 inch point
    // mostly retracted all the way to the frame
    controller
        .b()
        .onTrue(
            Commands.parallel(
                    // Tuning for climbing has more load and moves more quickly
                    new InstantCommand(() -> winchLeft.winchTuning(true), winchLeft),
                    new InstantCommand(() -> winchRight.winchTuning(true), winchRight))
                .andThen(
                    Commands.parallel(
                        new InstantCommand(() -> winchLeft.setGoalInch(25.5), winchLeft),
                        new InstantCommand(() -> winchRight.setGoalInch(25.5), winchRight)))); */

    /* // B button
    //
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true)); */
  }

  // Compute the pose where the robot should start a climb sequence
  // Find the nearest stage AprilTag to the current robot position
  // then lookup the pose of that tag and offset a desired distance
  // in front of that tag
  public Pose2d getNearestStagePose(boolean isRedAlliance, double offsetInches) {
    Pose2d botPose = drive.getPose();
    Pose2d tagPose;
    double distMin;
    if (isRedAlliance) {
      var tagPoseL = FieldConstants.getTagPose2d(11);
      var distL = botPose.minus(tagPoseL).getTranslation().getNorm();
      var tagPoseR = FieldConstants.getTagPose2d(12);
      var distR = botPose.minus(tagPoseR).getTranslation().getNorm();
      if (distL < distR) {
        tagPose = tagPoseL;
        distMin = distL;
      } else {
        tagPose = tagPoseR;
        distMin = distR;
      }
      var tagPoseC = FieldConstants.getTagPose2d(13);
      var distC = botPose.minus(tagPoseC).getTranslation().getNorm();
      if (distC < distMin) {
        tagPose = tagPoseC;
      }
    } else {
      var tagPoseL = FieldConstants.getTagPose2d(15);
      var distL = botPose.minus(tagPoseL).getTranslation().getNorm();
      var tagPoseR = FieldConstants.getTagPose2d(16);
      var distR = botPose.minus(tagPoseR).getTranslation().getNorm();
      if (distL < distR) {
        tagPose = tagPoseL;
        distMin = distL;
      } else {
        tagPose = tagPoseR;
        distMin = distR;
      }
      var tagPoseC = FieldConstants.getTagPose2d(14);
      var distC = botPose.minus(tagPoseC).getTranslation().getNorm();
      if (distC < distMin) {
        tagPose = tagPoseC;
      }
    }
    Transform2d offset =
        new Transform2d(
            new Translation2d(Units.inchesToMeters(offsetInches), 0), Rotation2d.fromDegrees(0));
    return tagPose.plus(offset);
  }

  // Compute the pose where the robot should start a climb sequence
  // Determine the AprilTag ID of the stage face to climb on
  // then lookup the pose of that tag and offset a desired distance
  // in front of that tag
  public Pose2d getStagePose(StageLocation stage, boolean isRedAlliance, double offsetInches) {
    int tagId = isRedAlliance ? stage.tagIdRed : stage.tagIdBlue;
    Pose2d tagPose =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(tagId).get().toPose2d();
    Transform2d offset =
        new Transform2d(
            new Translation2d(Units.inchesToMeters(offsetInches), 0), Rotation2d.fromDegrees(0));
    return tagPose.plus(offset);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // This is a good area to build more complex commands that are called by
  // controller button event triggers or other triggers

  // aimAtSpeaker will do these things in parallel
  // Spin up the shooter to the correct voltage based on distance to target
  // Raise the arm to the correct angle looked up based on distance to target (verify noteIsLoaded
  // before moving)
  // Rotate the chassis to point at the speaker using a biased rotation target based on angle to
  // speaker
  // Finish only when arm angle is at target and chassis angle is at target
  // Assume shooter will spin up before arm or chassis commands finish
  public Command aimAtSpeakerCommand() {
    return new ParallelCommandGroup(
        // Get the shooter running at the correct speed for the current distance away from the
        // speaker
        // based on pose
        new InstantCommand(
            () -> shooter.runVolts(12.0 * ShooterConstants.SPEED_MAP.get(drive.getDistToSpeaker())),
            shooter),
        // Raise the arm to the correct angle for the current distance away from the speaker based
        // on pose
        new FunctionalCommand(
                () -> arm.setGoalDeg(ShooterConstants.ANGLE_MAP.get(drive.getDistToSpeaker())),
                () -> {},
                (interrupted) -> {},
                () -> arm.atGoal(),
                arm)
            // By the way, don't start raising the arm unless a Note is ready to shoot
            .beforeStarting(
                new FunctionalCommand(
                    () -> {},
                    () -> {},
                    (interrupted) -> {},
                    () -> (convey.noteIsLoaded() || RobotBase.isSimulation()))),
        // Turn the base to the correct angle to shoot into the speaker
        turnToSpeakerCommand());
  }

  // prepareShooter will do two things in parallel after a Note is detected as being loaded
  // Raise the arm to the correct angle looked up based on distance to target
  // Spin up the shooter to the correct voltage based on distance to target
  // Distance to target needs to come from current PathPlanner path segment
  // Finish only when arm angle is at target
  // Assume shooter will spin up before arm command finishes
  public Command prepareShooterCommand() {
    return new SequentialCommandGroup(
        // wait until note is loaded
        new FunctionalCommand(
            () -> {},
            () -> {},
            (interrupted) -> {},
            () -> (convey.noteIsLoaded() || RobotBase.isSimulation())),
        // then raise the arm to the angle needed at path end
        new FunctionalCommand(
                () -> arm.setGoalDeg(ShooterConstants.ANGLE_MAP.get(drive.getPathEndToSpeaker())),
                () -> {},
                (interrupted) -> {},
                () -> arm.atGoal(),
                arm)
            // and in parallel, get the shooter running at the rpm needed at path end
            .alongWith(
                new InstantCommand(
                    () ->
                        shooter.runVolts(
                            12.0 * ShooterConstants.SPEED_MAP.get(drive.getPathEndToSpeaker())),
                    shooter)));
  }

  // prepareForAmp will do two things in parallel after a Note is detected as being loaded
  // Raise the arm to the initial angle for the Amp
  // Spin up the shooter to the correct voltage for the Amp
  // Finish only when arm angle is at target
  // Assume shooter will spin up before arm command finishes
  public Command prepareForAmpCommand() {
    return new SequentialCommandGroup(
        // wait until note is loaded
        new FunctionalCommand(
            () -> {},
            () -> {},
            (interrupted) -> {},
            () -> (convey.noteIsLoaded() || RobotBase.isSimulation())),
        // then raise the arm to the angle needed for the amp
        new FunctionalCommand(
                () -> arm.setGoalDeg(ArmConstants.ARM_AMP_ANGLE_DEG),
                () -> {},
                (interrupted) -> {},
                () -> arm.atGoal(),
                arm)
            // and in parallel, get the shooter running at the rpm needed for the amp
            .alongWith(
                new InstantCommand(
                    () -> shooter.runVolts(ShooterConstants.AMP_SHOOT_VOLTS), shooter)));
  }

  // Use getVectorFaceSpeaker to find the angle between the current robot pose
  // and the pose of the center of the speaker opening
  // Need to add PI and wrap if needed to get the correct angle to face the back
  // of the robot at the speaker
  // Finally, bias the angle based on the angle
  // When shooting at an angle, shoot beyond the center of the speaker opening
  // and use the hood to give more clearance past the near edge of the speaker opening
  public Command turnToSpeakerCommand() {
    return new TurnToAngleCommand(
        drive,
        () ->
            drive.getBiasedShootingRot(
                drive
                    .getVectorFaceSpeaker()
                    .getAngle()
                    .plus(new Rotation2d(Math.PI)) // shooter faces backwards
                    .getRadians()));
  }
  // To shoot is to run the conveyor to advance a Note into the shooter
  // Other logic needs to make sure the shooter is running and that a Note is present to shoot
  // Finish the command as soon as the Note is not detected by the sensor
  // The Note will have cleared the convey rollers by then
  // Add a timeout for some safety if the shooter is not running or a Note gets stuck in any other
  // way
  public Command shootCommand() {
    // Run the conveyor at close to max speed to push the Note into the already-running shooter
    // wheels
    return new StartEndCommand(() -> convey.runVolts(11.5), () -> convey.stop(), convey)
        // Stop conveying when the Note sensor no longer sees a Note
        .until(() -> !convey.noteIsLoaded())
        // By the way, in case the arm is not up, wait for it to be up before conveying
        .beforeStarting(new WaitUntilCommand(() -> arm.armIsUp()))
        // Stop conveying after some time even if the sensor still sees the Note - likely a problem
        .withTimeout(0.5)
        .withName("Shoot");
  }

  // The sticky polycarb Amp slot backing makes it necessary to do more than just shoot the Note
  // into the Amp
  // We found that we get good Amp deposits if we also
  // Rotate the arm forward another 10 degrees
  // While simultaneously slowly driving away from the Amp (forward, since intake faces away)
  // After depositing the Note, lower the arm and stop the shooter
  public Command depositAmpCommand() {
    // Two main groups of things to do
    return new SequentialCommandGroup(
        // Group 1 of things to do simultaneously
        new ParallelCommandGroup(
            // Push the arm forward even more than the fully up position - stop when it reaches goal
            new FunctionalCommand(
                () -> arm.setGoalDeg(ArmConstants.ARM_AMP_ANGLE2_DEG),
                () -> {},
                (interrupted) -> {},
                () -> arm.atGoal(),
                arm),
            // Drive slowly forward for 1/2 second in robot-centric - this is away from the Amp
            DriveCommands.driveByValues(drive, 0.08, 0.0, 0.0, () -> false).withTimeout(0.5),
            // Wait for 1/4 second for the arm and base to move, then shoot the Note into the Amp
            new SequentialCommandGroup(new WaitCommand(0.25), shootCommand())),
        // Group 2 of things to do after all the previous group finishes
        new ParallelCommandGroup(
            // Stop the drive - may not be necessary
            new InstantCommand(() -> drive.stop(), drive),
            // Stop the shooter
            new InstantCommand(() -> shooter.stop(), shooter),
            // Lower the arm to the loading angle - do not wait for it to finish
            // This overall command can finish immediately and the robot can get back to driving
            new InstantCommand(() -> arm.setGoalDeg(ArmConstants.ARM_LOAD_ANGLE_DEG), arm)));
  }

  public static final InterpolatingDoubleTreeMap CLIMB_MAP = new InterpolatingDoubleTreeMap();

  // curve to follow relating the arm angle to the distance driven out
  // from the sensed under-stage starting position in the pre-climb sequence
  static {
    CLIMB_MAP.put(Units.feetToMeters(0.0), 23.0);
    CLIMB_MAP.put(Units.feetToMeters(1.33), 63.0);
  }

  // Huge sequence of steps to take to accomplish a climb starting from
  // a known pose in front of the target trap
  public Command ClimbCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(intake::stop, intake),
            new InstantCommand(drive::setReferencePoint),
            new InstantCommand(() -> drive.isUsingVision = false)),
        // drive to clear chain before starting to look for the stage
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> drive.getMetersFromPoint() > Units.inchesToMeters(24.0)),
            // speed was -.19 for comp bot
            DriveCommands.driveByValues(drive, -0.125, 0.0, 0.0, () -> false)),
        // drive until sensing the stage (or a fixed distance in simulation)
        new ParallelDeadlineGroup(
            new WaitUntilCommand(
                () ->
                    drive.underStage()
                        || (Constants.currentMode == Constants.Mode.SIM
                            && drive.getMetersFromPoint() > Units.inchesToMeters(40.0))),
            // speed was -.19 for comp bot
            DriveCommands.driveByValues(drive, -0.125, 0.0, 0.0, () -> false)),
        // tuck Note into shooter and raise arm to pre-drive angle
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> arm.atGoal()),
            new InstantCommand(drive::stop, drive),
            new InstantCommand(() -> convey.runVolts(1.2), convey),
            new InstantCommand(() -> arm.setGoalDeg(23.0), arm)),
        // reset reference point for drive-out
        new InstantCommand(drive::setReferencePoint),
        // drive out step 1 and raise arm on the distance curve
        // raise the hooks
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> drive.getMetersFromPoint() > Units.inchesToMeters(5.9)),
            new InstantCommand(() -> convey.runVolts(0.0), convey),
            // speed was 0.10 for comp bot
            DriveCommands.driveByValues(drive, 0.06, 0.0, 0.0, () -> false),
            new InstantCommand(() -> winchLeft.setGoalInch(0.0), winchLeft),
            new InstantCommand(() -> winchRight.setGoalInch(0.0), winchRight),
            new RunCommand(() -> arm.setGoalDeg(CLIMB_MAP.get(drive.getMetersFromPoint())), arm)),
        // continue to drive out a little slower while raising the arm on the curve
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> drive.getMetersFromPoint() > Units.inchesToMeters(16.0)),
            // speed was 0.09 for comp bot
            DriveCommands.driveByValues(drive, 0.05, 0.0, 0.0, () -> false),
            new RunCommand(() -> arm.setGoalDeg(CLIMB_MAP.get(drive.getMetersFromPoint())), arm)),
        // start lowering the outriggers - servo internal controller handles the lowering
        new InstantCommand(
            () ->
                arm.setOutriggerPos(
                    ArmConstants.OUTRIGGER_LEFT_DOWN, ArmConstants.OUTRIGGER_RIGHT_DOWN)),
        // if hooks are not yet fully up, give time for that to complete
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> (winchLeft.atGoal() && winchRight.atGoal())),
            new InstantCommand(drive::stop, drive)),
        // drive far enough to latch hooks on chain and pull them off the uprights
        // also raise the arm to the final position for shooting in the trap
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> drive.getMetersFromPoint() > Units.inchesToMeters(27.0)),
            // speed was 0.15 for comp bot
            DriveCommands.driveByValues(drive, 0.10, 0.0, 0.0, () -> false),
            new InstantCommand(() -> arm.setGoalDeg(93.0), arm),
            new InstantCommand(() -> winchLeft.winchTuning(true), winchLeft),
            new InstantCommand(() -> winchRight.winchTuning(true), winchRight)),
        // climb by retracting both winches
        // in case something happens with the rope winding and the climbers cannot get to goal
        // use the X button to finish this group and move on to the shooting commands
        new ParallelCommandGroup(
            new InstantCommand(drive::stop, drive),
            new FunctionalCommand(
                () -> winchLeft.setGoalInch(25.5),
                () -> {},
                (interrupted) -> {},
                () -> (winchLeft.atGoal() || controller.x().getAsBoolean()),
                winchLeft),
            new FunctionalCommand(
                () -> winchRight.setGoalInch(25.5),
                () -> {},
                (interrupted) -> {},
                () -> (winchRight.atGoal() || controller.x().getAsBoolean()),
                winchRight)),
        // pass 1 at depositing note
        // run convey and run shooter at 50% top, -5% bottom
        // run this for 1/2 second
        new ParallelDeadlineGroup(
            new WaitCommand(0.5),
            new InstantCommand(() -> convey.runVolts(4.5), convey),
            new InstantCommand(() -> shooter.runVoltsEach(-0.6, 6.0), shooter)),
        // pass 2 at depositing note
        // in case pass 1 did not eject Note, run lower shooter axle at 20% forward
        // conveyor and shooter will keep running until FMS disables at end of match
        new InstantCommand(() -> shooter.runVoltsEach(2.4, 6.0), shooter));
  }

  // If there is anything that needs to happen when the robot enters disable,
  // auto, or teleop modes, those things can be defined here
  // This might be an easier place to make this happen rather than directly
  // in Robot.java, since all the subsystems already exist here
  public void disableInitialize() {
    arm.armClosedLoop = false;
  }

  public void autoInitialize() {
    arm.armClosedLoop = true;
  }

  public void teleopInitialize() {
    arm.armClosedLoop = true;
  }
}
