package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NoteCamera {

  PhotonCamera camera;
  String path;

  public NoteCamera(String name) {
    // This is a constructor
    camera = new PhotonCamera(name);
    path = "Vision/" + name.replace("_", "");
  }

  public PhotonTrackedTarget getTarget() {
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
    } else {
      Logger.recordOutput(path + "/Yaw", 0d);
      Logger.recordOutput(path + "/Pitch", 0d);
      Logger.recordOutput(path + "/Area", 0d);
      Logger.recordOutput(path + "/Skew", 0d);
    }

    return best;
  }

  public Optional<Double> getNoteYaw() {
    // This is a method
    var res = getTarget();
    if(res != null) {
      return Optional.of(res.getYaw());
    } else {
      return Optional.empty();
    }
  }
}
