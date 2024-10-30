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
import java.util.HashMap;
import java.util.Map;

// Place to hold constants that are used in multiple locations within the code
// Consider placing subsystem-only constants in the subsystem
// Constants should generally be declared public static final
public final class Constants {
  public static final Mode currentMode = Mode.REAL;
  public static final Integer NUM_CAMS = 1;

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

    public static final double DRIVE_CURRENT_LIMIT = 40.0;
    public static final int TURN_CURRENT_LIMIT = 30;

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

    // cam pose translation is relative to the center of the robot
    // x is front positive to back negative
    // y is left positive to right negative
    // z is distance from the floor - up is positive
    // rotation roll is if the camera is rotated about the axis pointing perpendicularly
    //   out from the lens - this should be 0 in most cases
    // pitch is the angle the camera is tilted up or down - negative is tilted up
    // yaw is the angle the camera is facing from the robot - 0 is front and angle increases CCW
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

    // standard deviations of pose estimates for Kalman filter fusing
    // cameras are relative to odometry
    // pick odometry base values and then set cameras at multiples
    // note that odometry angle (n3) is the gyro,
    //   so it should be very good relative to camera angle estimates
    // n1 and n2 are x and y translation and should generally be equal

    // reference odometry standard deviations are roughly in (meters, meters, radians)
    // could probably go with (1, 1, 1) and scale camera SDs from there, but a lot of example code
    // uses (0.05, 0.05, 0.05) and scales from that, so that is used here so our values are typical
    public static final Matrix<N3, N1> STD_DEVS_ODOMETRY = VecBuilder.fill(0.05, 0.05, 0.05);

    public static final Matrix<N3, N1> STD_DEVS_VISION_DEFAULT = VecBuilder.fill(1, 1, 1.5);
    public static final Matrix<N3, N1> STD_DEVS_SINGLE_TAG = VecBuilder.fill(1.5, 1.5, 8);
    public static final Matrix<N3, N1> STD_DEVS_MULTI_TAG = VecBuilder.fill(0.5, 0.5, 1);

    public static final double VISION_CUTOFF_DIST_M = 4.0;
    public static final double DIST_TO_DOUBLE_SD_M = 5.5;
    // multiply std_devs in auto to trust them less (>1) or more (<1)
    public static final double VISION_AUTO_MULTIPLIER = 1.0;

    public static final Map<Integer, String> TAG_DESC = new HashMap<Integer, String>();

    static {
      TAG_DESC.put(-1, "NotSet");
      TAG_DESC.put(1, "BlueSourceR");
      TAG_DESC.put(2, "BlueSourceL");
      TAG_DESC.put(3, "RedSpeakerR");
      TAG_DESC.put(4, "RedSpeakerL");
      TAG_DESC.put(5, "RedAmp");
      TAG_DESC.put(6, "BlueAmp");
      TAG_DESC.put(7, "BlueSpeakerR");
      TAG_DESC.put(8, "BlueSpeakerL");
      TAG_DESC.put(9, "RedSourceR");
      TAG_DESC.put(10, "RedSourceL");
      TAG_DESC.put(11, "RedStageLeft");
      TAG_DESC.put(12, "RedStageRight");
      TAG_DESC.put(13, "RedCenterStage");
      TAG_DESC.put(14, "BlueCenterStage");
      TAG_DESC.put(15, "BlueStageLeft");
      TAG_DESC.put(16, "BlueStageRight");
    }

    private VisionConstants() {}
  }

  public static class ArmConstants {
    public static final double ARM_LENGTH_IN = 20.0;
    public static final double ARM_MASS_LBF = 10.0;
    public static final double ARM_GEAR_REDUCTION = 355.56;
    // Arm angle is defined to be 0 when the arm is parallel to the floor
    public static final double ARM_MIN_ANGLE_DEG = -34.0;
    public static final double ARM_MAX_ANGLE_DEG = 95.0;
    public static final double ARM_TARGET_DEG = 90.0;
    public static final double ARM_AMP_ANGLE_DEG = 83.0;
    public static final double ARM_AMP_ANGLE2_DEG = 93.0;
    // how close to the PID goal to be considered at goal?
    public static final double ARM_ANGLE_TOLERANCE_DEG = 0.2;
    // how close to the load angle to be considered down?
    public static final double ARM_IS_DOWN_TOLERANCE_DEG = 1.0;
    // External encoder reading in degrees when the arm is parallel to the floor
    public static final double ARM_ENCODER_OFFSET_DEG = 187.0;

    // climb stabilization outriggers mounted to the base of the arm
    // extension amount is 0-1, where 0 is fully retracted
    public static final double OUTRIGGER_LEFT_UP = 0.0;
    public static final double OUTRIGGER_LEFT_DOWN = 1.0;
    public static final double OUTRIGGER_RIGHT_UP = 0.05;
    public static final double OUTRIGGER_RIGHT_DOWN = 0.97;

    private ArmConstants() {}
  }

  public static class WinchConstants {
    public static final double WINCH_GEAR_REDUCTION = 25.0;
    // tension from the gas spring
    public static final double WINCH_TENSION_LOAD_LB = 1.0;
    // each winch gets 1/2 the total robot weight
    public static final double WINCH_ROBOT_LOAD_LB = 117.0 / 2.0;
    public static final double WINCH_SPOOL_DIA_IN = 1.125;
    // winch heights are distance from the top (fully extended) position
    public static final double WINCH_MIN_HEIGHT_IN = 0.0;
    public static final double WINCH_MAX_HEIGHT_IN = 26.0;
    public static final double WINCH_START_HEIGHT_IN = 8.34;
    public static final int WINCH_CURRENT_LIMIT = 30;
    public static final double WINCH_HEIGHT_TOLERANCE_IN = 0.2;
    public static final double WINCH_VELOCITY_CLIMB_IPS = 4.0;
    public static final double WINCH_ACCEL_CLIMB_IPS2 = 40.0;
    public static final double WINCH_VELOCITY_EXTEND_IPS = 5.0;
    public static final double WINCH_ACCEL_EXTEND_IPS2 = 40.0;

    private WinchConstants() {}
  }

  public static class ShooterConstants {

    // shoot wider than the direct angle to the speaker Apriltag to give more clearance for the Note
    public static final double ANGLE_BIAS_MULTIPLIER = 1.1;
    public static final double AMP_SHOOT_VOLTS = 2.0;

    public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();

    // x: distance (meters) from rear camera lens to center of speaker opening projected down
    // This is how the data was collected
    // Code will take care of offset from camera lens to center of robot
    // y: shooter arm angle above horizontal (degrees)
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

    public static final InterpolatingDoubleTreeMap SPEED_MAP = new InterpolatingDoubleTreeMap();

    // x: distance (meters) from rear camera lens to center of speaker opening projected down
    // This is how the data was collected
    // Code will take care of offset from camera lens to center of robot
    // y: shooter motor fraction output - code will convert to motor volts
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

    public static final InterpolatingDoubleTreeMap LOB_ANGLE_MAP = new InterpolatingDoubleTreeMap();

    // x: distance (meters) from the center of the robot to the target point on the floor
    // y: arm angle above horizontal (degrees)
    static {
      LOB_ANGLE_MAP.put(Units.feetToMeters(10.0), 1.5);
      LOB_ANGLE_MAP.put(Units.feetToMeters(15.0), 3.75);
      LOB_ANGLE_MAP.put(Units.feetToMeters(20.0), 6.0);
      LOB_ANGLE_MAP.put(Units.feetToMeters(25.0), 8.25);
      LOB_ANGLE_MAP.put(Units.feetToMeters(30.0), 10.5);
    }

    public static final InterpolatingDoubleTreeMap LOB_SPEED_MAP = new InterpolatingDoubleTreeMap();

    // x: distance (meters) from the center of the robot to the target point on the floor
    // y: shooter motor volts
    static {
      LOB_SPEED_MAP.put(Units.feetToMeters(10.0), 3.3);
      LOB_SPEED_MAP.put(Units.feetToMeters(15.0), 4.0);
      LOB_SPEED_MAP.put(Units.feetToMeters(20.0), 4.8);
      LOB_SPEED_MAP.put(Units.feetToMeters(25.0), 5.5);
      LOB_SPEED_MAP.put(Units.feetToMeters(30.0), 6.3);
    }

    private ShooterConstants() {}
  }
}
