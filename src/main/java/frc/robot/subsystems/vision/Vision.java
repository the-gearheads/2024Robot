// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase {
  private PhotonCamera cameraLeft;
  private PhotonCamera cameraRight;
  private AprilTagFieldLayout field;
  private PoseStrategy strategy;
  private PhotonPoseEstimator leftEstimator;
  private PhotonPoseEstimator rightEstimator;

  public Vision() {
    cameraLeft = new PhotonCamera(LEFT_CAM_NAME);
    cameraRight = new PhotonCamera(RIGHT_CAM_NAME);
    
    try {
      field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("ERROR Opening apriltag field layout file");
      System.out.println(AprilTagFields.k2024Crescendo.m_resourceFile);
    }
    strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    leftEstimator = new PhotonPoseEstimator(field, strategy, cameraLeft, LEFT_TRANSFORM);
    rightEstimator = new PhotonPoseEstimator(field, strategy, cameraRight, RIGHT_TRANSFORM);
  }

  public Optional<EstimatedRobotPose> getGlobalPoseFromLeft() {return leftEstimator.update();}
  public Optional<EstimatedRobotPose> getGlobalPoseFromRight() {return rightEstimator.update();}

}