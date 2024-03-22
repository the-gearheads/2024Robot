package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.FieldConstants.FIELD;
import static frc.robot.Constants.VisionConstants.*;

public class Camera {

  public final String name;
  public final String path;
  public final Transform3d transform;
  public final CameraIntrinsics intrinsics;

  public final PhotonCamera camera;
  public final PhotonPoseEstimator estimator;

  private static final double MAX_PITCHROLL = Units.degreesToRadians(10);
  private static final double MAX_Z = Units.inchesToMeters(12);


  public Camera(AprilTagFieldLayout field, String name, Transform3d transform, CameraIntrinsics intrinsics) {
    this.name = name;
    this.transform = transform;
    this.intrinsics = intrinsics;
    path = "Vision/" + name;

    camera = new PhotonCamera(name);

    var strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    estimator = new PhotonPoseEstimator(field, strategy, camera, transform);
  }

  public Optional<EstimatedRobotPose> getGlobalPose() {
    var pipelineResult = camera.getLatestResult();
    return filterPose(estimator.update(pipelineResult), pipelineResult);
  }

  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose, PhotonPipelineResult result) {
    if(!pose.isPresent()) return Optional.empty();
    Pose3d estPose = pose.get().estimatedPose;
    
    double pitch = estPose.getRotation().getX();
    double roll = estPose.getRotation().getY();
    Logger.recordOutput(path + "/EstPoseUnfiltered", estPose);
    if (Math.abs(pitch) > MAX_PITCHROLL || Math.abs(roll) > MAX_PITCHROLL || estPose.getTranslation().getZ() > MAX_Z) {
      return Optional.empty();
    }
    if (!FIELD.contains(estPose.toPose2d())) {
      return Optional.empty();
    }
    Logger.recordOutput(path + "/EstPose", estPose);
    return pose;
  }

  public boolean feedPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
    var pose = getGlobalPose();
    if(pose.isPresent()) {
        Pose2d estPose = pose.get().estimatedPose.toPose2d();
        Matrix<N3, N1> stddevs = pose.get().targetsUsed.size() <= 1 ? SINGLE_TAG_STD_DEVS : MULTI_TAG_STD_DEVS;
        poseEstimator.addVisionMeasurement(estPose, pose.get().timestampSeconds, stddevs);
        return true;
      }
    return false;
  }

  public SimCameraProperties getSimProperties() {
    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(1280, 720, intrinsics.getCameraMatrix(), intrinsics.getDistCoeffs());

    // Approximate detection noise with average and standard deviation error in pixels.
    properties.setCalibError(0.02, 0.05);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    properties.setFPS(30);
    // The average and standard deviation in milliseconds of image data latency.
    properties.setAvgLatencyMs(35);
    properties.setLatencyStdDevMs(7);

    return properties;
  }
}
