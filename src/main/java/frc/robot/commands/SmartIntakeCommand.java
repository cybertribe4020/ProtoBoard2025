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
    // if the arm is down and a Note has not tripped the sensor in the shooter
    // run both intake axles at their intaking speed and direction
    if (armIsDown && !noteIsLoaded) {
      intake.runVolts(intakeVoltsInput.get(), intakeVoltsInput.get());
      // intake.runVelocity(intakeVelocityInput.get(), intakeVelocityInput.get());
      intakeDirection = "intake";
    } else {
      // if arm is up, run lower/back axle stopped and upper/front axle backwards
      // this will hopefully keep extra Notes out of the robot if one is already present
      intake.runVolts(0.0, -3.0);
      intakeDirection = "reject";
    }
    Logger.recordOutput("Intake/intakeDirection", intakeDirection);
  }

  public void end(boolean interrupted) {
    intake.stop();
  }
}
