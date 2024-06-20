package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.convey.Convey;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class SmartConveyCommand extends Command {

  private final Convey convey;
  private final BooleanSupplier armIsDownSupplier;
  private final BooleanSupplier noteIsLoadedSupplier;
  private final BooleanSupplier shooterIsRunningSupplier;
  private final BooleanSupplier leftTriggerPulledSupplier;

  public SmartConveyCommand(
      Convey convey,
      BooleanSupplier armIsDownSupplier,
      BooleanSupplier noteIsLoadedSupplier,
      BooleanSupplier shooterIsRunningSupplier,
      BooleanSupplier leftTriggerPulledSupplier) {
    this.convey = convey;
    this.armIsDownSupplier = armIsDownSupplier;
    this.noteIsLoadedSupplier = noteIsLoadedSupplier;
    this.shooterIsRunningSupplier = shooterIsRunningSupplier;
    this.leftTriggerPulledSupplier = leftTriggerPulledSupplier;
  }

  public void initialize() {}

  public void execute() {
    var armIsDown = armIsDownSupplier.getAsBoolean();
    var noteIsLoaded = noteIsLoadedSupplier.getAsBoolean();
    var shooterIsRunning = shooterIsRunningSupplier.getAsBoolean();
    var leftTriggerPulled = leftTriggerPulledSupplier.getAsBoolean();
    var conveyStatus = "undefined";
    if (leftTriggerPulled && shooterIsRunning) {
      convey.runVolts(11.5);
      conveyStatus = "shooting";
    } else if (armIsDown && !noteIsLoaded) {
      convey.runVolts(4.0);
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
