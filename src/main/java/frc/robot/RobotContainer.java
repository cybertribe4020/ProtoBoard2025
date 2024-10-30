package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SmartMaxVelocity1Command;
import frc.robot.commands.SmartMaxVelocity2Command;
import frc.robot.subsystems.arm1.Arm1;
import frc.robot.subsystems.arm1.Arm1IOSparkFlex;
import frc.robot.subsystems.maxVelocity1.MaxVelocity1;
import frc.robot.subsystems.maxVelocity1.MaxVelocity1IOSparkMax;
import frc.robot.subsystems.maxVelocity2.MaxVelocity2;
import frc.robot.subsystems.maxVelocity2.MaxVelocity2IOSparkMax;

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

public class RobotContainer {
  // Subsystems
  private final MaxVelocity1 maxVelocity1;
  private final MaxVelocity2 maxVelocity2;
  private final Arm1 arm;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    maxVelocity1 = new MaxVelocity1(new MaxVelocity1IOSparkMax());
    maxVelocity2 = new MaxVelocity2(new MaxVelocity2IOSparkMax());
    arm = new Arm1(new Arm1IOSparkFlex());

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Right Bumper
    // Run the intake and conveyor
    // For now, this is set to toggle to be able to turn these motors off while testing
    controller.leftBumper().toggleOnTrue(new SmartMaxVelocity1Command(maxVelocity1));
    controller.rightBumper().toggleOnTrue(new SmartMaxVelocity2Command(maxVelocity2));
    controller.x().toggleOnTrue(maxVelocity1.loadCommand());
    controller.y().onTrue(arm.armToTargetCommand());
    controller.a().onTrue(arm.armToZeroCommand());
  }
}
