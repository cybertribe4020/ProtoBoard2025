package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.convey.Convey;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

// Handles the running of the Note conveyor so the driver doesn't have to
// Runs the conveyor at high speed when the driver asks to shoot and the shooter
// wheels are running
// Runs the conveyor at regular speed to move a note from the intake into the shooter
// if the arm is down in loading position and the note sensor before the shooter wheels
// does not detect a note
// Stops the conveyor in any other situation

public class SmartConveyCommand extends Command {

  private final Convey convey;
  private final BooleanSupplier armIsDownSupplier;
  private final BooleanSupplier noteIsLoadedSupplier;
  private final BooleanSupplier shooterIsRunningSupplier;
  private final BooleanSupplier shootSignalSupplier;
  // private LoggedDashboardNumber conveyVoltsInput = new LoggedDashboardNumber("Convey Volts",
  // 4.0);
  private LoggedDashboardNumber conveyVelocityInput = new LoggedDashboardNumber("Convey RPM", 900);

  public SmartConveyCommand(
      Convey convey,
      BooleanSupplier armIsDownSupplier,
      BooleanSupplier noteIsLoadedSupplier,
      BooleanSupplier shooterIsRunningSupplier,
      BooleanSupplier shootSignalSupplier) {
    this.convey = convey;
    this.armIsDownSupplier = armIsDownSupplier;
    this.noteIsLoadedSupplier = noteIsLoadedSupplier;
    this.shooterIsRunningSupplier = shooterIsRunningSupplier;
    this.shootSignalSupplier = shootSignalSupplier;
  }

  public void initialize() {}

  public void execute() {
    var armIsDown = armIsDownSupplier.getAsBoolean();
    var noteIsLoaded = noteIsLoadedSupplier.getAsBoolean();
    var shooterIsRunning = shooterIsRunningSupplier.getAsBoolean();
    var shootTriggerPulled = shootSignalSupplier.getAsBoolean();
    var conveyStatus = "undefined";
    // Shoot by running the conveyor near max speed when driver pulls the trigger
    if (shootTriggerPulled && shooterIsRunning) {
      convey.runVolts(12.0);
      conveyStatus = "shooting";
      // Run the conveyor at normal convey speed to accept a Note from the intake
      // if the arm is down and a Note has not tripped the loading sensor in the shooter
    } else if (armIsDown && !noteIsLoaded) {
      // convey.runVolts(conveyVoltsInput.get());
      convey.runVelocity(conveyVelocityInput.get());
      conveyStatus = "runToLoad";
      // in any other state stop the conveyor
    } else {
      convey.runVolts(0.0);
      conveyStatus = "stopped";
    }
    Logger.recordOutput("Convey/conveyStatus", conveyStatus);
  }

  public void end(boolean interrupted) {
    convey.stop();
  }
}
