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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Intake intake;
  private final Convey convey;
  private final Arm1 arm;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber shooterSpeedInput =
      new LoggedDashboardNumber("Shooter RPM", 1500.0);
  private final LoggedDashboardNumber intakeSpeedInput =
      new LoggedDashboardNumber("Intake RPM", 1300);
  private final LoggedDashboardNumber conveySpeedInput =
      new LoggedDashboardNumber("Convey RPM", 900);
  private LoggedDashboardNumber intakeVoltsInput = new LoggedDashboardNumber("Intake Volts", 4.0);
  private LoggedDashboardNumber conveyVoltsInput = new LoggedDashboardNumber("Convey Volts", 4.0);

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
        break;
    }

    // Set up auto routines
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

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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

    /* autoChooser.addOption(
        "Shooter SysId (Quasistatic Forward)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Quasistatic Reverse)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse)); */

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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // X button
    // Stop the drivetrain and turn the wheels to an X position
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // B button
    //
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Left Bumper
    // Run the intake and conveyor
    // For now, this is set to toggle to be able to turn these motors off while testing
    controller
        .leftBumper()
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
                drive::getVectorFaceSpeaker));

    // Right Trigger
    // Run convey motor to shoot a Note
    // Shooter must be running before this action runs the convey motor
    controller
        .rightTrigger(0.5)
        .and(() -> shooter.shooterIsRunning())
        .whileTrue(Commands.startEnd(() -> convey.runVolts(11.5), convey::stop, convey));

    // Y Button
    // Drive directly to climb start location in front of red stage left
    // Will not avoid any Stage legs in the way!!
    controller
        .y()
        .whileTrue(
            new DriveToPoseCommand(
                drive,
                drive::getPose, // could also use () -> drive.getPose()
                new Pose2d(12.6, 2.4, Rotation2d.fromDegrees(300.0)),
                false));

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
                        new Pose2d(12.6, 2.4, Rotation2d.fromDegrees(300.0)),
                        false)));

    // POV Left
    // Test turning to shooting angle
    controller
        .povLeft()
        .onTrue(
            new TurnToAngleCommand(
                    drive,
                    () ->
                        drive.getBiasedShootingRot(
                            drive
                                .getVectorFaceSpeaker()
                                .getAngle()
                                .plus(new Rotation2d(Math.PI)) // shooter faces backwards
                                .getRadians()))
                .withTimeout(2.0));

    // POV Down
    controller.povDown().onTrue(shootCommand());

    // POV Up
    controller.povUp().onTrue(aimAtSpeakerCommand());

    // POV Right
    // Stop arm
    controller.povRight().onTrue(Commands.startEnd(arm::stop, arm::stop, arm));

    // Back button
    // Toggle the use of vision for pose estimation
    controller
        .back()
        .onTrue(
            Commands.runOnce(() -> drive.isUsingVision = !drive.isUsingVision, drive)
                .ignoringDisable(true));

    // Left stick button
    // Drive robot centric
    controller
        .leftStick()
        .onTrue(
            Commands.runOnce(() -> drive.driveFieldCentric = false, drive).ignoringDisable(true));

    // Right stick button
    // Drive field centric
    controller
        .rightStick()
        .onTrue(
            Commands.runOnce(() -> drive.driveFieldCentric = true, drive).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

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
        new InstantCommand(
            () -> shooter.runVolts(12.0 * ShooterConstants.SPEED_MAP.get(drive.getDistToSpeaker())),
            shooter),
        new FunctionalCommand(
                () -> arm.setGoalDeg(ShooterConstants.ANGLE_MAP.get(drive.getDistToSpeaker())),
                () -> {},
                (interrupted) -> {},
                () -> arm.atGoal(),
                arm)
            .beforeStarting(
                new FunctionalCommand(
                    () -> {},
                    () -> {},
                    (interrupted) -> {},
                    () -> (convey.noteIsLoaded() || RobotBase.isSimulation()))),
        new TurnToAngleCommand(
            drive,
            () ->
                drive.getBiasedShootingRot(
                    drive
                        .getVectorFaceSpeaker()
                        .getAngle()
                        .plus(new Rotation2d(Math.PI)) // shooter faces backwards
                        .getRadians())));
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
    return new StartEndCommand(() -> convey.runVolts(11.5), () -> convey.stop(), convey)
        .until(() -> !convey.noteIsLoaded())
        .beforeStarting(new WaitUntilCommand(() -> arm.armIsUp()))
        .withTimeout(0.5)
        .withName("Shoot");
  }

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
