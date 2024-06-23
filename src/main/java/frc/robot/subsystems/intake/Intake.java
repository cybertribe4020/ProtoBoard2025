// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
// vision
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0032);
        io.configurePID(0.0001, 0.0, 0.0);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0032);
        io.configurePID(0.0001, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0032);
        io.configurePID(0.0001, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Stops the intake. */
  public void stop() {
    io.stop();
  }

  /** Run both axles (lower/upper) open loop at the specified voltages. */
  public void runVolts(double voltsLower, double voltsUpper) {
    io.setVoltage(voltsLower, voltsUpper);
  }

  /** Run both axles (lower/upper) closed loop at the specified velocities. */
  public void runVelocity(double velocityRPMLower, double velocityRPMUpper) {
    io.setVelocity(
        velocityRPMLower,
        ffModel.calculate(velocityRPMLower),
        velocityRPMUpper,
        ffModel.calculate(velocityRPMUpper));

    // Log intake setpoint
    Logger.recordOutput("Intake/SetpointRPMLower", velocityRPMLower);
    Logger.recordOutput("Intake/SetpointRPMUpper", velocityRPMUpper);
    Logger.recordOutput("Intake/ffVoltsLower", ffModel.calculate(velocityRPMLower));
    Logger.recordOutput("Intake/ffVoltsUpper", ffModel.calculate(velocityRPMUpper));
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPMLower() {
    return inputs.velocityRPM[0];
  }

  @AutoLogOutput
  public double getVelocityRPMUpper() {
    return inputs.velocityRPM[1];
  }

  public void smartIntakeControl (
    double intakeRPMFwd,
    double intakeRPMRev,
    BooleanSupplier armIsDownSupplier,
    BooleanSupplier noteIsLoadedSupplier) {

    var intakeDirection = "undefined";
    if (armIsDownSupplier.getAsBoolean() && !noteIsLoadedSupplier.getAsBoolean()) {
      this.runVelocity(intakeRPMFwd, intakeRPMFwd);
      intakeDirection = "intake";
    } else {
      // if arm is up, run lower axle off and upper axle in reverse
      this.runVelocity(0.0, intakeRPMRev);
      intakeDirection = "reverse";
    }
    Logger.recordOutput("Intake/intakeDirection", intakeDirection);
  }
}
