package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class SmartIntakeCommand extends Command {

  private final Intake intake;
  private final BooleanSupplier armIsDownSupplier;
  private final BooleanSupplier noteIsLoadedSupplier;
  private LoggedDashboardNumber intakeVoltsInput = new LoggedDashboardNumber("Intake Volts", 4.0);

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
      // intake.runVolts(intakeVoltsInput.get(), intakeVoltsInput.get());
      intake.runVelocity(1200, 1200);
      intakeDirection = "intake";
    } else {
      intake.runVolts(-intakeVoltsInput.get() * 0.75, 0.0);
      intakeDirection = "reject";
    }
    Logger.recordOutput("Intake/intakeDirection", intakeDirection);
  }

  public void end(boolean interrupted) {
    intake.stop();
  }
}
