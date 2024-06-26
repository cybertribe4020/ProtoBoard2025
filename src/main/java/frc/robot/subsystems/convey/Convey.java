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

package frc.robot.subsystems.convey;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Convey extends SubsystemBase {
  private final ConveyIO io;
  private final ConveyIOInputsAutoLogged inputs = new ConveyIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final DigitalInput conveyNoteSensor = new DigitalInput(9);

  public LoggedDashboardNumber conveyVelocityInput = new LoggedDashboardNumber("Convey RPM", 900);

  /** Creates a new Convey. */
  public Convey(ConveyIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0043);
        io.configurePID(0.00005, 0.0, 0.0);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0043);
        io.configurePID(0.00005, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0043);
        io.configurePID(0.00005, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Convey", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    io.setVelocity(velocityRPM, ffModel.calculate(velocityRPM));

    // Log convey setpoint
    Logger.recordOutput("Convey/SetpointRPM", velocityRPM);
    Logger.recordOutput("Convey/ffVolts", ffModel.calculate(velocityRPM));
  }

  /** Stops the convey. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRPM;
  }

  // The note sensor reads True without a note present and False when it sees a note
  @AutoLogOutput
  public boolean noteIsLoaded() {
    return !conveyNoteSensor.get();
  }

  // To shoot is to run the conveyor to advance a Note into the shooter
  // Other logic needs to make sure the shooter is running and that a Note is present to shoot
  // Finish the command as soon as the Note is not detected by the sensor
  // The Note will have cleared the convey rollers by then
  // Add a timeout for some safety if the shooter is not running or a Note gets stuck in any other
  // way
  public Command shootCommand() {
    return new StartEndCommand(() -> runVolts(11.5), () -> stop(), this)
        .until(() -> !noteIsLoaded())
        .withTimeout(0.5)
        .withName("Shoot");
  }

  public Command loadCommand() {
    return new StartEndCommand(() -> runVelocity(conveyVelocityInput.get()), () -> stop(), this)
        .until(() -> (noteIsLoaded() || RobotBase.isSimulation()))
        .withName("Load");
  }
}
