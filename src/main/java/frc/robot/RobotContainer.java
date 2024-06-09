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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoseCommand;
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
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
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
  private final Flywheel flywheel;
  private final Intake intake;
  private final Convey convey;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);
  private final LoggedDashboardNumber intakeSpeedInput =
      new LoggedDashboardNumber("Intake Speed", 1600.0);
  private final LoggedDashboardNumber conveySpeedInput =
      new LoggedDashboardNumber("Convey Speed", 1600.0);
  private final LoggedDashboardNumber intakeVoltsInput =
      new LoggedDashboardNumber("Intake Volts", 4.0);
  private final LoggedDashboardNumber conveyVoltsInput =
      new LoggedDashboardNumber("Convey Volts", 4.0);

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
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        convey = new Convey(new ConveyIOSparkMax());
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
        flywheel = new Flywheel(new FlywheelIOSim());
        intake = new Intake(new IntakeIOSim());
        convey = new Convey(new ConveyIOSim());
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
        flywheel = new Flywheel(new FlywheelIO() {});
        intake = new Intake(new IntakeIO() {});
        convey = new Convey(new ConveyIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "runFlywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(15.0));
    NamedCommands.registerCommand(
        "runIntake",
        Commands.startEnd(
                () -> intake.runVolts(intakeVoltsInput.get(), intakeVoltsInput.get()),
                intake::stop,
                intake)
            .withTimeout(15.0));
    NamedCommands.registerCommand(
        "runConvey",
        Commands.startEnd(() -> convey.runVolts(conveyVoltsInput.get()), convey::stop, convey)
            .withTimeout(15.0));
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
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    controller
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                Commands.startEnd(
                    () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel),
                Commands.startEnd(
                    () -> intake.runVelocity(intakeSpeedInput.get(), intakeSpeedInput.get()),
                    intake::stop,
                    intake),
                Commands.startEnd(
                    () -> convey.runVelocity(conveySpeedInput.get()), convey::stop, convey)));

    // drive to climb start location in front of red stage left
    controller
        .y()
        .whileTrue(
            new DriveToPoseCommand(
                drive,
                drive::getPose, // could also use () -> drive.getPose()
                new Pose2d(12.6, 2.4, Rotation2d.fromDegrees(300.0)),
                false));

    // drive a path with obstacle avoidance to climb start location in front of red stage left
    // note that the pathplanner only gets within a "navgrid" resolution of the target pose
    // need to finish with a final driveToPose to fully get to the target pose
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

    // toggle use of vision for pose estimation
    controller
        .back()
        .onTrue(
            Commands.runOnce(() -> drive.isUsingVision = !drive.isUsingVision, drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
