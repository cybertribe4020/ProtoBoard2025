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

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim simLower = new FlywheelSim(DCMotor.getNEO(1), 24.0 / 16.0, 0.00015);
  private FlywheelSim simUpper = new FlywheelSim(DCMotor.getNEO(1), 24.0 / 16.0, 0.00015);
  private PIDController pidLower = new PIDController(0.0, 0.0, 0.0);
  private PIDController pidUpper = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVoltsLower = 0.0;
  private double ffVoltsUpper = 0.0;
  private double appliedVoltsLower = 0.0;
  private double appliedVoltsUpper = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (closedLoop) {
      appliedVoltsLower =
          MathUtil.clamp(
              pidLower.calculate(simLower.getAngularVelocityRPM()) + ffVoltsLower, -12.0, 12.0);
      simLower.setInputVoltage(appliedVoltsLower);
      appliedVoltsUpper =
          MathUtil.clamp(
              pidUpper.calculate(simUpper.getAngularVelocityRPM()) + ffVoltsUpper, -12.0, 12.0);
      simUpper.setInputVoltage(appliedVoltsUpper);
    }

    simLower.update(0.02);
    simUpper.update(0.02);

    inputs.positionRot = new double[] {0.0, 0.0};
    inputs.velocityRPM =
        new double[] {simLower.getAngularVelocityRPM(), simUpper.getAngularVelocityRPM()};
    inputs.appliedVolts = new double[] {appliedVoltsLower, appliedVoltsUpper};
    inputs.currentAmps =
        new double[] {simLower.getCurrentDrawAmps(), simUpper.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double voltsLower, double voltsUpper) {
    closedLoop = false;
    appliedVoltsLower = voltsLower;
    appliedVoltsUpper = voltsUpper;
    simLower.setInputVoltage(voltsLower);
    simUpper.setInputVoltage(voltsUpper);
  }

  @Override
  public void setVelocity(
      double velocityRPMLower, double ffVoltsLower, double velocityRPMUpper, double ffVoltsUpper) {
    closedLoop = true;
    pidLower.setSetpoint(velocityRPMLower);
    pidUpper.setSetpoint(velocityRPMUpper);
    this.ffVoltsLower = ffVoltsLower;
    this.ffVoltsUpper = ffVoltsUpper;
  }

  @Override
  public void stop() {
    setVoltage(0.0, 0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidLower.setPID(kP, kI, kD);
    pidUpper.setPID(kP, kI, kD);
  }
}
