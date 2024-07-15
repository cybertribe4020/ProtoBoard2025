package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOPhoton implements CameraIO {
  protected final PhotonCamera camera;
  protected final PhotonPoseEstimator photonEstimator;
  protected double lastEstTimestamp = 0;
  protected Transform3d cameraPose;
  private final RawSubscriber rawBytesSubscriber;
  protected String cameraName;
  private Drive drive;

  // The layout of the AprilTags on the field
  public AprilTagFieldLayout tagLayout = (AprilTagFields.kDefaultField.loadAprilTagLayoutField());

  public CameraIOPhoton(Drive robotDrive, int index) {
    drive = robotDrive;

    switch (index) {
      default:
        cameraName = CAM_NAME_REAR;
        cameraPose = CAM_POSE_REAR;
        break;
      case 1:
        cameraName = CAM_NAME_LEFT;
        cameraPose = CAM_POSE_LEFT;
        break;
      case 2:
        cameraName = CAM_NAME_RIGHT;
        cameraPose = CAM_POSE_RIGHT;
        break;
    }

    camera = new PhotonCamera(cameraName);

    photonEstimator =
        new PhotonPoseEstimator(
            tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraPose);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    rawBytesSubscriber =
        NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable(cameraName)
            .getRawTopic("rawBytes")
            .subscribe(
                "rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult pipelineResult) {
    // Optional<EstimatedRobotPose> visionEst;

    return photonEstimator.update(pipelineResult);
  }

  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, PhotonPipelineResult cameraResult) {
    var estStdDevs = STD_DEVS_SINGLE_TAG;
    var targets = cameraResult.getTargets();
    int numTags = 0;

    double avgDist = 0;

    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0) return estStdDevs;

    avgDist /= numTags;
    Logger.recordOutput("Vision/avgDistance", avgDist);

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = STD_DEVS_MULTI_TAG;

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 32));

    if (DriverStation.isAutonomous()) {
      estStdDevs = estStdDevs.times(VISION_AUTO_MULTIPLIER);
    }

    return estStdDevs;
  }

  @Override
  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    if (Constants.currentMode != Constants.Mode.REPLAY) {

      TimestampedRaw[] rawBytesFrames = rawBytesSubscriber.readQueue();
      inputs.numFrames = rawBytesFrames.length;
      inputs.rawBytes = new byte[inputs.numFrames][];
      inputs.timestamps = new double[inputs.numFrames];

      for (int n = 0; n < rawBytesFrames.length; n++) {
        inputs.rawBytes[n] = rawBytesFrames[n].value;
        inputs.timestamps[n] = rawBytesFrames[n].timestamp / 1e6;
      }
    }

    if (inputs.numFrames > 0) {

      byte[] latestFrame = inputs.rawBytes[inputs.numFrames - 1];

      var pipelineResult = PhotonPipelineResult.serde.unpack(new Packet(latestFrame));

      pipelineResult.setTimestampSeconds(
          inputs.timestamps[inputs.numFrames - 1] - pipelineResult.getLatencyMillis() / 1e3);

      var visionEst = getEstimatedGlobalPose(pipelineResult);

      if (Constants.currentMode != Constants.Mode.REPLAY) {
        inputs.latestTimestamp = pipelineResult.getTimestampSeconds();
        inputs.photonLatency = pipelineResult.getLatencyMillis() / 1e3;
        inputs.poseDetected = false; // Could be changed below
      }

      if (visionEst.isPresent()) {
        var photonPoseEst = visionEst.get();
        var estPose2d = photonPoseEst.estimatedPose.toPose2d();
        var latestTimestamp = pipelineResult.getTimestampSeconds();

        if (Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5) {
          lastEstTimestamp = latestTimestamp;
          var estStdDevs = this.getEstimationStdDevs(estPose2d, pipelineResult);
          drive.addVisionMeasurement(estPose2d, photonPoseEst.timestampSeconds, estStdDevs);

          if (Constants.currentMode != Constants.Mode.REPLAY) {
            inputs.poseDetected = true;
          }

          double robotToCamXYDiff = drive.getPose().minus(estPose2d).getTranslation().getNorm();
          int diffHue = Math.max((int) (60 - (robotToCamXYDiff / 0.3 * 60.0)), 0);
          Logger.recordOutput("Vision/" + cameraName + "/robotToCam", robotToCamXYDiff);
          Logger.recordOutput(
              "Vision/" + cameraName + "/robotToCamColor",
              Color.fromHSV(diffHue, 200, 230).toHexString());

          Logger.recordOutput("Vision/" + cameraName, estPose2d);
          // Logger.recordOutput("Vision/" + cameraName + "/rawBytes", latestFrame);
        }
      } else {
        Logger.recordOutput("Vision/" + cameraName + "/robotToCam", Double.NaN);
        Logger.recordOutput("Vision/" + cameraName + "/robotToCamColor", "#222222");
      }

      if (pipelineResult.hasTargets()) {
        var target = pipelineResult.getBestTarget();
        Logger.recordOutput(
            "Vision/" + cameraName + "/id", VisionConstants.TAG_DESC.get(target.getFiducialId()));
        Logger.recordOutput("Vision/" + cameraName + "/ambiguity", target.getPoseAmbiguity());
        Logger.recordOutput(
            "Vision/" + cameraName + "/distance",
            target.getBestCameraToTarget().getTranslation().getNorm());
      } else {
        Logger.recordOutput("Vision/" + cameraName + "/id", "NoTag");
        Logger.recordOutput("Vision/" + cameraName + "/ambiguity", Double.NaN);
        Logger.recordOutput("Vision/" + cameraName + "/distance", Double.NaN);
      }
    }
  }
}
