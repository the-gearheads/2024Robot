package frc.robot.subsystems.vision;


import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NoteCamera {

  PhotonCamera camera;
  String path;

  private final double MIN_AREA = 10;

  public NoteCamera(String name) {
    // This is a constructor
    camera = new PhotonCamera(name);
    path = "Vision/" + name.replace("_", "");
  }

  public PhotonTrackedTarget getTarget() {
    if(1==1) return null;
    PhotonPipelineResult res = camera.getLatestResult();
    int numTargets = res.getTargets().size();
    Logger.recordOutput(path + "/Valid", numTargets > 0);
    Logger.recordOutput(path + "/NumTargets", numTargets);

    PhotonTrackedTarget bestNoteTarget = numTargets > 0 ? res.getTargets().get(0) : null;

    if(bestNoteTarget != null) {
      Logger.recordOutput(path + "/Note", bestNoteTarget.getBestCameraToTarget());
      Logger.recordOutput(path + "/Yaw", bestNoteTarget.getYaw());
      Logger.recordOutput(path + "/Pitch", bestNoteTarget.getPitch());
      Logger.recordOutput(path + "/Area", bestNoteTarget.getArea());
      Logger.recordOutput(path + "/Skew", bestNoteTarget.getSkew());
      Logger.recordOutput(path + "C3Y", bestNoteTarget.getDetectedCorners().get(3).y);

      if(bestNoteTarget.getArea() < MIN_AREA) {
        return null;
      }
    } else {
      Logger.recordOutput(path + "/Yaw", 0d);
      Logger.recordOutput(path + "/Pitch", 0d);
      Logger.recordOutput(path + "/Area", 0d);
      Logger.recordOutput(path + "/Skew", 0d);
    }

    return bestNoteTarget;
  }
}
