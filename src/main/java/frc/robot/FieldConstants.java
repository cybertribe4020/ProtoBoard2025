package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {

  public static boolean isBlue() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Blue)
        .orElse(true);
  }

  // field position in pretty much all standard software for FRC is relative
  // to an origin where field endline on the blue side intersects with the
  // right sideline as viewed from the blue driver stations
  // when field elements are reflectively symmetric about the field center line
  // you often need to "flip" the X coordinate when you are on the red alliance
  public static boolean shouldFlip() {
    return (DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red)
        .orElse(true));
  }

  public static double getFlip() {
    return isBlue() ? 1 : -1;
  }

  // position of the center of the speaker opening projected to the floor
  // returned as a Translation2d object
  public static Translation2d redSpeakerPosition =
      AprilTagFields.k2024Crescendo
          .loadAprilTagLayoutField()
          .getTagPose(4)
          .get()
          .getTranslation()
          .toTranslation2d();
  public static Translation2d blueSpeakerPosition =
      AprilTagFields.k2024Crescendo
          .loadAprilTagLayoutField()
          .getTagPose(7)
          .get()
          .getTranslation()
          .toTranslation2d();

  public static Translation2d getSpeakerPosition() {
    if (isBlue()) {
      return blueSpeakerPosition;
    } else {
      return redSpeakerPosition;
    }
  }

  // location of the "center" speaker AprilTag returned as a Pose2d object
  public static Pose2d getSpeakerPose2d() {
    if (isBlue()) {
      return AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(7).get().toPose2d();
    } else {
      return AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(4).get().toPose2d();
    }
  }

  // location of an AprilTag ID returned as a Pose2d object
  public static Pose2d getTagPose2d(int tagId) {
    return AprilTagFields.k2024Crescendo
        .loadAprilTagLayoutField()
        .getTagPose(tagId)
        .get()
        .toPose2d();
  }

  // position of the center of the shooter exit projected to the floor
  // if the robot bumper is touching the subwoofer base and the robot
  // is centered side-to-side on the subwoofer
  // returned as a Translation2d object
  public static Translation2d getSubwooferPosition() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return redSpeakerPosition.minus(
          new Translation2d(Units.feetToMeters(4.083), 0).minus(shooterPositionOffset));
    }
    return blueSpeakerPosition.plus(
        new Translation2d(Units.feetToMeters(4.083), 0).plus(shooterPositionOffset));
  }

  public static Translation2d shooterPositionOffset = new Translation2d(0, 0);

  // return AprilTag IDs for red and blue for a given face of the stage
  public static enum StageLocation {
    RIGHT(12, 16),
    LEFT(11, 15),
    MIDDLE(13, 14);
    public int tagIdRed, tagIdBlue;

    StageLocation(int red, int blue) {
      this.tagIdBlue = blue;
      this.tagIdRed = red;
    }
  }
}
