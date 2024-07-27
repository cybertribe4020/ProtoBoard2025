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

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private FlywheelSim simLower = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.00059);
  private FlywheelSim simUpper = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.00059);
  private PIDController pidLower = new PIDController(0.0, 0.0, 0.0);
  private PIDController pidUpper = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVoltsLower = 0.0;
  private double appliedVoltsUpper = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      appliedVoltsLower =
          MathUtil.clamp(
              pidLower.calculate(simLower.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      simLower.setInputVoltage(appliedVoltsLower);
      appliedVoltsUpper =
          MathUtil.clamp(
              pidUpper.calculate(simUpper.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      simUpper.setInputVoltage(appliedVoltsUpper);
    }

    simLower.update(0.02);
    simUpper.update(0.02);

    inputs.positionLowerRad = 0.0;
    inputs.velocityLowerRadPerSec = simLower.getAngularVelocityRadPerSec();
    inputs.appliedVoltsLower = appliedVoltsLower;
    inputs.currentAmpsLower = simLower.getCurrentDrawAmps();
    inputs.positionUpperRad = 0.0;
    inputs.velocityUpperRadPerSec = simUpper.getAngularVelocityRadPerSec();
    inputs.appliedVoltsUpper = appliedVoltsUpper;
    inputs.currentAmpsUpper = simUpper.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVoltsLower = volts;
    simLower.setInputVoltage(volts);
    appliedVoltsUpper = volts;
    simUpper.setInputVoltage(volts);
  }

  @Override
  public void setVoltageEach(double voltsLower, double voltsUpper) {
    closedLoop = false;
    appliedVoltsLower = voltsLower;
    simLower.setInputVoltage(voltsLower);
    appliedVoltsUpper = voltsUpper;
    simUpper.setInputVoltage(voltsUpper);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pidLower.setSetpoint(velocityRadPerSec);
    pidUpper.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidLower.setPID(kP, kI, kD);
    pidUpper.setPID(kP, kI, kD);
  }
}
