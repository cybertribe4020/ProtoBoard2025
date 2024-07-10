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

import static java.lang.Math.PI;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;
  public static final Integer NUM_CAMS = 2;

  public static enum Mode {
    REAL, // running on a real robot
    SIM, // running on a physics simulator
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
    public static final double MAX_ANGULAR_SPEED =
        MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS; // radians per second

    private DriveConstants() {}
  }

  public static class AutoConstants {

    public static final double X_kP = 5.0;
    public static final double X_kI = 0.0;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 5.0;
    public static final double Y_kI = 0.0;
    public static final double Y_kD = 0.0;

    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
        new TrapezoidProfile.Constraints(PI, 2 / PI);
    public static final double THETA_kP = 6.0;
    public static final double THETA_kI = 0.02;
    public static final double THETA_kD = 0.0;

    public static final double PATH_THETA_kP = 2.6;
    public static final double PATH_THETA_kI = 0.001;
    public static final double PATH_THETA_kD = 0.0;

    private AutoConstants() {}
  }

  public static class VisionConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.223); // length in meters
    public static final double FIELD_WIDTH = Units.inchesToMeters(323.277); // length in meters

    public static final String CAM_NAME_REAR = "aprilTagData";
    public static final String CAM_NAME_LEFT = "aprilTagLeft";
    public static final String CAM_NAME_RIGHT = "aprilTagRight";

    public static final Transform3d CAM_POSE_REAR =
        new Transform3d(
            new Translation3d(-10.0, 0.0, 17.0).times(Units.inchesToMeters(1.0)),
            new Rotation3d(0.0, Math.toRadians(-40.0), Units.degreesToRadians(182.0)));
    public static final Transform3d CAM_POSE_LEFT =
        new Transform3d(
            new Translation3d(-11.125, 7.44, 14.25).times(Units.inchesToMeters(1.0)),
            new Rotation3d(0.0, Math.toRadians(-26.0), Units.degreesToRadians(90.0)));
    // x is -10.375 if mount is shifted towards back by one hole, -9.875 if flush with arm upright
    public static final Transform3d CAM_POSE_RIGHT =
        new Transform3d(
            new Translation3d(-10.375, -7.44, 14.25).times(Units.inchesToMeters(1.0)),
            new Rotation3d(0.0, Math.toRadians(-26.0), Units.degreesToRadians(270.0)));

    public static final Matrix<N3, N1> STD_DEVS_VISION_DEFAULT = VecBuilder.fill(1, 1, 1.5);
    public static final Matrix<N3, N1> STD_DEVS_SINGLE_TAG = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> STD_DEVS_MULTI_TAG = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> STD_DEVS_ODOMETRY = VecBuilder.fill(0.05, 0.05, 0.05);

    public static final double VISION_AUTO_MULTIPLIER =
        1.0; // multiply std_devs in auto to trust them less (>1) or more (<1)

    private VisionConstants() {}
  }

  public static class ArmConstants {
    public static final double ARM_LENGTH_IN = 20.0;
    public static final double ARM_MASS_LBF = 10.0;
    public static final double ARM_GEAR_REDUCTION = 355.56;
    // Arm angle is defined to be 0 when the arm is parallel to the floor
    public static final double ARM_MIN_ANGLE_DEG = -34.0;
    public static final double ARM_MAX_ANGLE_DEG = 95.0;
    public static final double ARM_LOAD_ANGLE_DEG = -31.0;
    public static final double ARM_AMP_ANGLE_DEG = 83.0;
    public static final double ARM_AMP_ANGLE2_DEG = 93.0;
    public static final double ARM_LOB_ANGLE_DEG = 8.0;
    // External encoder reading in degrees when the arm is parallel to the floor
    public static final double ARM_ENCODER_OFFSET_DEG = 187.0;

    private ArmConstants() {}
  }

  public static class ShooterConstants {

    // shoot wider than the direct angle to the speaker Apriltag to give more clearance for the Note
    public static final double ANGLE_BIAS_MULTIPLIER = 1.1;
    public static final double AMP_SHOOT_VOLTS = 2.0;
    public static final double LOB_SHOOT_VOLTS = 8.0;

    // x: distance (meters) from rear camera lens to center of speaker opening projected down
    // Code will take care of offset from camera lens to center of robot
    // y: shooter arm angle above horizontal (degrees)
    public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();

    static {
      ANGLE_MAP.put(Units.feetToMeters(0.0), 5.0);
      ANGLE_MAP.put(Units.feetToMeters(4.0), 6.0);
      ANGLE_MAP.put(Units.feetToMeters(5.0), 11.5);
      ANGLE_MAP.put(Units.feetToMeters(6.0), 16.0);
      ANGLE_MAP.put(Units.feetToMeters(7.0), 20.0);
      ANGLE_MAP.put(Units.feetToMeters(8.0), 22.5);
      ANGLE_MAP.put(Units.feetToMeters(9.0), 25.25);
      ANGLE_MAP.put(Units.feetToMeters(10.0), 27.25);
      ANGLE_MAP.put(Units.feetToMeters(11.0), 29.0);
      ANGLE_MAP.put(Units.feetToMeters(12.0), 30.5);
      ANGLE_MAP.put(Units.feetToMeters(13.0), 32.25);
      ANGLE_MAP.put(Units.feetToMeters(14.0), 33.5);
      ANGLE_MAP.put(Units.feetToMeters(15.0), 31.0);
      ANGLE_MAP.put(Units.feetToMeters(16.0), 32.0);
    }

    // x: distance (meters) from rear camera lens to center of speaker opening projected down
    // Code will take care of offset from camera lens to center of robot
    // y: shooter motor fraction output - code will convert to motor volts
    public static final InterpolatingDoubleTreeMap SPEED_MAP = new InterpolatingDoubleTreeMap();

    static {
      SPEED_MAP.put(Units.feetToMeters(0.0), 0.53);
      SPEED_MAP.put(Units.feetToMeters(4.0), 0.53);
      SPEED_MAP.put(Units.feetToMeters(5.0), 0.53);
      SPEED_MAP.put(Units.feetToMeters(6.0), 0.55);
      SPEED_MAP.put(Units.feetToMeters(7.0), 0.57);
      SPEED_MAP.put(Units.feetToMeters(8.0), 0.60);
      SPEED_MAP.put(Units.feetToMeters(9.0), 0.63);
      SPEED_MAP.put(Units.feetToMeters(10.0), 0.66);
      SPEED_MAP.put(Units.feetToMeters(11.0), 0.70);
      SPEED_MAP.put(Units.feetToMeters(12.0), 0.73);
      SPEED_MAP.put(Units.feetToMeters(13.0), 0.76);
      SPEED_MAP.put(Units.feetToMeters(14.0), 0.79);
      SPEED_MAP.put(Units.feetToMeters(15.0), 0.79);
      SPEED_MAP.put(Units.feetToMeters(16.0), 0.81);
    }

    private ShooterConstants() {}
  }
}
