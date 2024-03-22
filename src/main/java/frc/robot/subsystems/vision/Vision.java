// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.Constants.FieldConstants.FIELD;
import static frc.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase {
  private PhotonCamera cameraFrontLeft;
  private PhotonCamera cameraFrontRight;
  private PhotonCamera cameraBackLeft;
  private AprilTagFieldLayout field;
  private PoseStrategy strategy;
  private PhotonPoseEstimator frontLeftEstimator;
  private PhotonPoseEstimator frontRightEstimator;
  private PhotonPoseEstimator backLeftEstimator;
  private VisionSim sim;
  private Swerve swerve;

  private static final double MAX_PITCHROLL = Units.degreesToRadians(10);
  private static final double MAX_Z = Units.inchesToMeters(12);

  public Vision(Swerve swerve) {
    cameraFrontLeft = new PhotonCamera(FRONT_LEFT_NAME);
    cameraFrontRight = new PhotonCamera(FRONT_RIGHT_NAME);
    cameraBackLeft = new PhotonCamera(BACK_LEFT_NAME);
    this.swerve = swerve;
    // might want to remove this before comp
    if(Robot.isSimulation())
      PhotonCamera.setVersionCheckEnabled(false);

    try {
      field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("ERROR Opening apriltag field layout file");
      System.out.println(AprilTagFields.k2024Crescendo.m_resourceFile);
    }
    strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    // strategy = PoseStrategy.LOWEST_AMBIGUITY;

    frontLeftEstimator = new PhotonPoseEstimator(field, strategy, cameraFrontLeft, FRONT_LEFT_TRANSFORM);
    frontRightEstimator = new PhotonPoseEstimator(field, strategy, cameraFrontRight, FRONT_RIGHT_TRANSFORM);
    backLeftEstimator = new PhotonPoseEstimator(field, strategy, cameraBackLeft, FRONT_RIGHT_TRANSFORM);
    sim = new VisionSim(cameraFrontLeft, cameraFrontRight, cameraBackLeft);
  }

  public Optional<EstimatedRobotPose> getGlobalPoseFrontLeft() {
    return getGlobalPose(frontLeftEstimator, FRONT_LEFT_NAME);
  }

  public Optional<EstimatedRobotPose> getGlobalPoseFrontRight() {
    return getGlobalPose(frontRightEstimator, FRONT_LEFT_NAME);
  }

  public Optional<EstimatedRobotPose> getGlobalPoseBackLeft() {
    return getGlobalPose(backLeftEstimator, BACK_LEFT_NAME);
  }
  
  private Optional<EstimatedRobotPose> getGlobalPose(PhotonPoseEstimator estimator, String cameraName) {
    Logger.recordOutput("Vision/" + cameraName + "/RefPose", estimator.getReferencePose());
    var updated = estimator.update();
    if(!updated.isPresent()) return Optional.empty();
    Pose3d estPose = updated.get().estimatedPose;
    double pitch = estPose.getRotation().getX();
    double roll = estPose.getRotation().getY();
    Logger.recordOutput("Vision/" + cameraName + "/EstPoseUnfiltered", estPose);
    if (Math.abs(pitch) > MAX_PITCHROLL || Math.abs(roll) > MAX_PITCHROLL || estPose.getTranslation().getZ() > MAX_Z) {
      return Optional.empty();
    }
    if (!FIELD.contains(estPose.toPose2d())) {
      return Optional.empty();
    }
    Logger.recordOutput("Vision/" + cameraName + "/EstPose", estPose);
    return updated;
  }

  @Override
  public void periodic() {
    sim.periodic(swerve.getPoseWheelsOnly());
  }
}
