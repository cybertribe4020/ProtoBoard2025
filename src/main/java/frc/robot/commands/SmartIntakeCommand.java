package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class SmartIntakeCommand extends Command {

  private final Intake intake;
  private final BooleanSupplier armIsDownSupplier;
  private final BooleanSupplier noteIsLoadedSupplier;

  public SmartIntakeCommand(
      Intake intake, BooleanSupplier armIsDownSupplier, BooleanSupplier noteIsLoadedSupplier) {
    this.intake = intake;
    this.armIsDownSupplier = armIsDownSupplier;
    this.noteIsLoadedSupplier = noteIsLoadedSupplier;
  }

  public void initialize() {}

  public void execute() {
    var armIsDown = armIsDownSupplier.getAsBoolean();
    var noteIsLoaded = noteIsLoadedSupplier.getAsBoolean();
    var intakeDirection = "undefined";
    if (armIsDown && !noteIsLoaded) {
      intake.runVelocity(1600.0, 1600.0);
      intakeDirection = "intake";
    } else {
      intake.runVelocity(-1000.0, 0);
      intakeDirection = "reject";
    }
    Logger.recordOutput("Intake/intakeDirection", intakeDirection);
  }

  public void end(boolean interrupted) {
    intake.stop();
  }
}
