package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.photonvision.simulation.VisionSystemSim;

public class Vision extends SubsystemBase {
  private final Integer numCams = 1; // update this when adding more cameras
  private final Camera[] cameras = new Camera[numCams];

  private Drive drive;
  private VisionSystemSim visionSim;

  // The layout of the AprilTags on the field
  public AprilTagFieldLayout tagLayout = (AprilTagFields.kDefaultField.loadAprilTagLayoutField());

  public Vision(Drive robotDrive) {
    drive = robotDrive;

    if (Constants.currentMode == Constants.Mode.SIM) { // SIM only, not REPLAY
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(tagLayout);

      for (int n = 0; n < numCams; n++) {
        cameras[n] = new Camera(drive, n, visionSim);
      }
    } else {
      for (int n = 0; n < numCams; n++) {
        cameras[n] = new Camera(drive, n);
      }
    }
  }

  @Override
  public void periodic() {
    for (int n = 0; n < numCams; n++) {
      cameras[n].periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    if (Constants.currentMode == Constants.Mode.SIM) { // Not REPLAY
      visionSim.update(drive.getPose());

      var debugField = getSimDebugField();
      debugField.getObject("EstimatedRobot").setPose(drive.getPose());
      debugField.getObject("EstimatedRobotModules").setPoses(drive.getModulePoses());
    }
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Constants.currentMode == Constants.Mode.SIM) {
      visionSim.resetRobotPose(pose);
    }
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return visionSim.getDebugField();
    } else {
      return null;
    }    
  }
}
