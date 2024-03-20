package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;

public class VisionSim {

  VisionSystemSim sim = new VisionSystemSim("main");

  SimCameraProperties frontLeftCameraProp = new SimCameraProperties();
  SimCameraProperties frontRightCameraProp = new SimCameraProperties();
  SimCameraProperties backLeftCameraProp = new SimCameraProperties();
  double fx = 737.6136442454854;
  double fy = 733.1927575565593;
  double cx = 662.3371068271363;
  double cy = 435.9984845786;
  double[] distCoeffs = {0.15288116557227518,-0.2878953642242236,-0.0010986978034486703,0.0011333394853758716,0.12276685039910991};

  PhotonCameraSim frontLeftSim;
  PhotonCameraSim frontRightSim;
  PhotonCameraSim backLeftSim;

  public VisionSim(PhotonCamera frontLeftCamera, PhotonCamera frontRightCamera, PhotonCamera backLeftCamera) {
    if(Robot.isReal()) return;
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    sim.addAprilTags(tagLayout);
    configCameraProp(frontLeftCameraProp);
    configCameraProp(frontRightCameraProp);

    frontLeftSim = new PhotonCameraSim(frontLeftCamera, frontLeftCameraProp);
    frontRightSim = new PhotonCameraSim(frontRightCamera, frontRightCameraProp);
    backLeftSim = new PhotonCameraSim(backLeftCamera, backLeftCameraProp);

    // frontLeftSim.enableDrawWireframe(true); // apparently resource intensive
    // frontRightSim.enableDrawWireframe(true);

    sim.addCamera(frontLeftSim, FRONT_LEFT_TRANSFORM);
    sim.addCamera(frontRightSim, FRONT_RIGHT_TRANSFORM);
    
    frontLeftSim.setMinTargetAreaPixels(1000);
    frontRightSim.setMinTargetAreaPixels(1000);
  }

  private void configCameraProp(SimCameraProperties camera) {
    camera.setCalibration(1280, 720, MatBuilder.fill(
      Nat.N3(), Nat.N3(),
      fx,  0.0, cx,
      0.0, fy,  cy,
      0.0, 0.0, 1.0
    ),
    VecBuilder.fill(distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4]));

    // Approximate detection noise with average and standard deviation error in pixels.
    camera.setCalibError(0.02, 0.05);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    camera.setFPS(30);
    // The average and standard deviation in milliseconds of image data latency.
    camera.setAvgLatencyMs(35);
    camera.setLatencyStdDevMs(7);

  }

  public void periodic(Pose2d robotPose) {
    if(Robot.isReal()) return;
    sim.update(robotPose);
  }
}
