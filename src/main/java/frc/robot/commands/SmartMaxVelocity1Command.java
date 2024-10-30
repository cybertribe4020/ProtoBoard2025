package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.maxVelocity1.MaxVelocity1;
// Handles the running of the Note maxVelocity1or so the driver doesn't have to
// Runs the conveyor at high speed when the driver asks to shoot and the shooter
// wheels are running
// Runs the conveyor at regular speed to move a note from the intake into the shooter
// if the arm is down in loading position and the note sensor before the shooter wheels
// does not detect a note
// Stops the conveyor in any other situation

public class SmartMaxVelocity1Command extends Command {

  private final MaxVelocity1 maxVelocity1;
  // private LoggedDashboardNumber maxVelocity1VoltsInput = new LoggedDashboardNumber("MaxVelocity1
  // Volts",
  // 4.0);

  public SmartMaxVelocity1Command(MaxVelocity1 maxVelocity1) {
    this.maxVelocity1 = maxVelocity1;
  }

  public void initialize() {}

  public void execute() {
    maxVelocity1.runVolts(12.0);
  }
  // Run the conveyor at normal maxVelocity1 speed to accept a Note from the intake
  // if the arm is down and a Note has not tripped the loading sensor in the shooter
  public void end(boolean interrupted) {
    maxVelocity1.stop();
  }
}
