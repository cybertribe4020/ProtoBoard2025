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

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double positionLowerRad = 0.0;
    public double velocityLowerRadPerSec = 0.0;
    public double appliedVoltsLower = 0.0;
    public double currentAmpsLower = 0.0;
    public double positionUpperRad = 0.0;
    public double velocityUpperRadPerSec = 0.0;
    public double appliedVoltsUpper = 0.0;
    public double currentAmpsUpper = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run open loop at the specified voltage per axle. */
  public default void setVoltageEach(double voltsLower, double voltsUpper) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
