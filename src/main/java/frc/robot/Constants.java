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

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import static java.lang.Math.PI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    REAL, // running on a real robot
    SIM,  // running on a physics simulator
    REPLAY // replaying from a log file
  }

  public static class DriveConstants {
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16.6); // meters per second
    public static final double TRACK_WIDTH_X =
        Units.inchesToMeters(24.75); // meters - 5.25 inches less than frame width for MK4i
    public static final double TRACK_WIDTH_Y =
        Units.inchesToMeters(23.75); // meters - 5.25 inches less than frame length for MK4i
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0); // meters
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS; // radians per second

    private DriveConstants() {

    }
  }

  public static class AutoConstants {
    
    public static final double X_kP = 5.0;
    public static final double X_kI = 0.0;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 5.0;
    public static final double Y_kI = 0.0;
    public static final double Y_kD = 0.0;

    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(PI, 2 / PI);
    public static final double THETA_kP = 6.0;
    public static final double THETA_kI = 0.02;
    public static final double THETA_kD = 0.0;

    public static final double PATH_THETA_kP = 2.6;
    public static final double PATH_THETA_kI = 0.001;
    public static final double PATH_THETA_kD = 0.0;

    private AutoConstants() {

    }
  }

  public static class VisionConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.223); // length in meters
    public static final double FIELD_WIDTH = Units.inchesToMeters(323.277); // length in meters

    private VisionConstants() {

    }
  }
}
