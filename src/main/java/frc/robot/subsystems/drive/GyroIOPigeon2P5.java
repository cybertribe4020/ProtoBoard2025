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

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
@SuppressWarnings({"deprecation", "removal"})
public class GyroIOPigeon2P5 implements GyroIO {
  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(43);

  public GyroIOPigeon2P5() {
    pigeon.setYaw(0.0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true; // not sure how to check connectedness in Phoenix5
    inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw());
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(
            pigeon.getRate()); // getRate is positive for clockwise rotations if Pigeon case top
    // points up
  }
}
