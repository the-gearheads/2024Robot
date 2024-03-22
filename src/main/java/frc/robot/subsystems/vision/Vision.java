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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

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

  final double MAX_PITCHROLL = Units.degreesToRadians(7);
  final double MAX_Z = Units.inchesToMeters(6);

  public Optional<EstimatedRobotPose> getGlobalPoseFrontLeft() {
    Logger.recordOutput("Vision/FrontLeft/RefPose", frontLeftEstimator.getReferencePose());
    var updated = frontLeftEstimator.update();
    if(updated.isPresent()) {
      double pitch = updated.get().estimatedPose.getRotation().getX();
      double roll = updated.get().estimatedPose.getRotation().getY();
      if(Math.abs(pitch) > MAX_PITCHROLL || Math.abs(roll) > MAX_PITCHROLL || updated.get().estimatedPose.getTranslation().getZ() > MAX_Z) {
        Logger.recordOutput("Vision/FrontLeft/EstPoseUnfiltered", updated.get().estimatedPose);
        return Optional.empty();
      }

      Logger.recordOutput("Vision/FrontLeft/EstPoseUnfiltered", updated.get().estimatedPose);
      Logger.recordOutput("Vision/FrontLeft/EstPose", updated.get().estimatedPose);
    }

    return updated;
  }

  public Optional<EstimatedRobotPose> getGlobalPoseFrontRight() {
    Logger.recordOutput("Vision/FrontRight/RefPose", frontRightEstimator.getReferencePose());
    var updated = frontRightEstimator.update();
    if(updated.isPresent()) {
      double pitch = updated.get().estimatedPose.getRotation().getX();
      double roll = updated.get().estimatedPose.getRotation().getY();
      if(Math.abs(pitch) > MAX_PITCHROLL || Math.abs(roll) > MAX_PITCHROLL || updated.get().estimatedPose.getTranslation().getZ() > MAX_Z) {
        Logger.recordOutput("Vision/FrontRight/EstPoseUnfiltered", updated.get().estimatedPose);
        return Optional.empty();
      }

      Logger.recordOutput("Vision/FrontRight/EstPoseUnfiltered", updated.get().estimatedPose);
      Logger.recordOutput("Vision/FrontRight/EstPose", updated.get().estimatedPose);
    }

    return updated;
  }

  public Optional<EstimatedRobotPose> getGlobalPoseBackLeft() {
    Logger.recordOutput("Vision/BackLeft/RefPose", backLeftEstimator.getReferencePose());
    var updated = backLeftEstimator.update();
    if(updated.isPresent()) {
      double pitch = updated.get().estimatedPose.getRotation().getX();
      double roll = updated.get().estimatedPose.getRotation().getY();
      if(Math.abs(pitch) > MAX_PITCHROLL || Math.abs(roll) > MAX_PITCHROLL || updated.get().estimatedPose.getTranslation().getZ() > MAX_Z) {
        Logger.recordOutput("Vision/BackLeft/EstPoseUnfiltered", updated.get().estimatedPose);
        return Optional.empty();
      }

      Logger.recordOutput("Vision/BackLeft/EstPoseUnfiltered", updated.get().estimatedPose);
      Logger.recordOutput("Vision/BackLeft/EstPose", updated.get().estimatedPose);
    }

    return updated;
  }

  private static final double xyStdDevCoefficient = 0.005;
  private static final double thetaStdDevCoefficient = 0.01;
  private static final double coefficientFactor = 1.0;
  

  private void updateSingleCamera(SwerveDrivePoseEstimator singleTagPoseEstimator, PhotonCamera camera, EstimatedRobotPose pose, String name) {
    var updated = camera.getLatestResult();
    int numTargets = updated.targets.size();
    double avgDistToTarget = 0;
    for(var target: updated.targets) {
      avgDistToTarget += target.getBestCameraToTarget().getTranslation().getNorm(); 
    }
    avgDistToTarget /= numTargets;

    double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistToTarget, 2.0) / numTargets * coefficientFactor;
    double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistToTarget, 2.0) / numTargets * coefficientFactor;

    Logger.recordOutput("Vision/" + name + "/XyStdDev", xyStdDev);
    Logger.recordOutput("Vision/" + name + "/ThetaStdDev", thetaStdDev);
    Logger.recordOutput("Vision/" + name + "/NumTargets", numTargets);
    Logger.recordOutput("Vision/" + name + "/AvgDistToTarget", avgDistToTarget);

    var stdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), xyStdDev, xyStdDev, thetaStdDev);

    singleTagPoseEstimator.addVisionMeasurement(
      pose.estimatedPose.toPose2d(),
      updated.getTimestampSeconds(), 
      stdDevs
    );

    // Log tag poses
    List<Transform3d> allTagPoses = new ArrayList<>();
    var currentPose = singleTagPoseEstimator.getEstimatedPosition();
    var currentPose3d = new Transform3d(new Translation3d(currentPose.getX(), currentPose.getY(), 0), new Rotation3d(0, 0, currentPose.getRotation().getRadians()));
    for (var detectionEntry: updated.targets) {
      var detection = detectionEntry.getBestCameraToTarget();
      var tagPose = currentPose3d.plus(detection);
      allTagPoses.add(tagPose);
    }
    Logger.recordOutput("Vision/" + name + "/TagPoses", allTagPoses.toArray(Transform3d[]::new));
  }

  public void updateSingleTagPoseEstimator(SwerveDrivePoseEstimator singleTagPoseEstimator, Optional<EstimatedRobotPose> frontRightPose, Optional<EstimatedRobotPose> frontLeftPose, Optional<EstimatedRobotPose> backLeftPose) {
    if(frontLeftPose.isPresent()) {
      updateSingleCamera(singleTagPoseEstimator, cameraFrontLeft, frontLeftPose.get(), "FrontLeft");
    } else {
      Logger.recordOutput("Vision/FrontLeft/NumTargets", 0);
      Logger.recordOutput("Vision/FrontLeft/TagPoses", new Transform3d[0]);
    }
    if(frontRightPose.isPresent()) {
      updateSingleCamera(singleTagPoseEstimator, cameraFrontRight, frontRightPose.get(), "FrontRight");
    } else {
      Logger.recordOutput("Vision/FrontRight/NumTargets", 0);
      Logger.recordOutput("Vision/FrontRight/TagPoses", new Transform3d[0]);
    }
    if(backLeftPose.isPresent()) {
      updateSingleCamera(singleTagPoseEstimator, cameraFrontRight, backLeftPose.get(), "BackLeft");
    } else {
      Logger.recordOutput("Vision/BackLeft/NumTargets", 0);
      Logger.recordOutput("Vision/BackLeft/TagPoses", new Transform3d[0]);
    }
  }

  @Override
  public void periodic() {
    sim.periodic(swerve.getPoseWheelsOnly());
  }
}
