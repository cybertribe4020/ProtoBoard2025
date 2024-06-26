package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.convey.Convey;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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
    var leftTriggerPulled = shootSignalSupplier.getAsBoolean();
    var conveyStatus = "undefined";
    if (leftTriggerPulled && shooterIsRunning) {
      convey.runVolts(12.0);
      conveyStatus = "shooting";
    } else if (armIsDown && !noteIsLoaded) {
      // convey.runVolts(conveyVoltsInput.get());
      convey.runVelocity(conveyVelocityInput.get());
      conveyStatus = "runToLoad";
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
