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

  public static boolean shouldFlip() {
    return (DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red)
        .orElse(true));
  }

  public static double getFlip() {
    return isBlue() ? 1 : -1;
  }

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

  public static Pose2d getSpeakerPose2d() {
    if (isBlue()) {
      return AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(7).get().toPose2d();
    } else {
      return AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(4).get().toPose2d();
    }
  }

  public static Translation2d shooterPositionOffset = new Translation2d(0, 0);

  public static Translation2d getSubwooferPosition() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return redSpeakerPosition.minus(
          new Translation2d(Units.feetToMeters(4.083), 0).minus(shooterPositionOffset));
    }
    return blueSpeakerPosition.plus(
        new Translation2d(Units.feetToMeters(4.083), 0).plus(shooterPositionOffset));
  }

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
