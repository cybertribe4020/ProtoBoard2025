package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.maxVelocity2.MaxVelocity2;
// Handles the running of the Note maxVelocity1or so the driver doesn't have to
// Runs the conveyor at high speed when the driver asks to shoot and the shooter
// wheels are running
// Runs the conveyor at regular speed to move a note from the intake into the shooter
// if the arm is down in loading position and the note sensor before the shooter wheels
// does not detect a note
// Stops the conveyor in any other situation

public class SmartMaxVelocity2Command extends Command {

  private final MaxVelocity2 maxVelocity2;
  // private LoggedDashboardNumber maxVelocity1VoltsInput = new LoggedDashboardNumber("MaxVelocity1
  // Volts",
  // 4.0);

  public SmartMaxVelocity2Command(MaxVelocity2 maxVelocity2) {
    this.maxVelocity2 = maxVelocity2;
  }

  public void initialize() {}

  public void execute() {
    maxVelocity2.runVolts(12.0);
  }
  // Run the conveyor at normal maxVelocity1 speed to accept a Note from the intake
  // if the arm is down and a Note has not tripped the loading sensor in the shooter
  public void end(boolean interrupted) {
    maxVelocity2.stop();
  }
}
