package frc.robot.subsystems.vision;


import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
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
    var res = camera.getLatestResult();
    int numTargets = res.getTargets().size();
    Logger.recordOutput(path + "/Valid", numTargets > 0);
    Logger.recordOutput(path + "/NumTargets", numTargets);

    var best = numTargets > 0 ? res.getTargets().get(0) : null;

    if(best != null) {
      Logger.recordOutput(path + "/Note", best.getBestCameraToTarget());
      Logger.recordOutput(path + "/Yaw", best.getYaw());
      Logger.recordOutput(path + "/Pitch", best.getPitch());
      Logger.recordOutput(path + "/Area", best.getArea());
      Logger.recordOutput(path + "/Skew", best.getSkew());
      Logger.recordOutput(path + "C3Y", best.getDetectedCorners().get(3).y);

      if(best.getArea() < MIN_AREA) {
        return null;
      }
    } else {
      Logger.recordOutput(path + "/Yaw", 0d);
      Logger.recordOutput(path + "/Pitch", 0d);
      Logger.recordOutput(path + "/Area", 0d);
      Logger.recordOutput(path + "/Skew", 0d);
    }

    return best;
  }
}
