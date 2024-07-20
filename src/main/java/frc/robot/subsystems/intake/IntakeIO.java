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

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Note that the 0 list position is the lower/back roller and 1 is upper/front
    public double[] positionRot = new double[] {};
    public double[] velocityRPM = new double[] {};
    public double[] appliedVolts = new double[] {};
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run each axle (lower/upper) open loop at the specified voltage. */
  public default void setVoltage(double voltsLower, double voltsUpper) {}

  /** Run each axle (lower/upper) closed loop at the specified velocity. */
  public default void setVelocity(
      double velocityRPMLower, double ffVoltsLower, double velocityRPMUpper, double ffVoltsUpper) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
