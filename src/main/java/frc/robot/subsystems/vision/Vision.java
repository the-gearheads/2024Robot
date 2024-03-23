// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout field;
  private VisionSim sim;
  private Swerve swerve;

  private final Camera frontLeft, frontRight, backLeft;

  CameraIntrinsics intrinsics = new CameraIntrinsics(
    737.6136442454854, 733.1927575565593, 662.3371068271363, 435.9984845786,
    new double[] {0.15288116557227518,-0.2878953642242236,-0.0010986978034486703,0.0011333394853758716,0.12276685039910991}
  );

  public Vision(Swerve swerve) {
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

    frontLeft = new Camera(field, FRONT_LEFT_NAME, FRONT_LEFT_TRANSFORM, intrinsics);
    frontRight = new Camera(field, FRONT_RIGHT_NAME, FRONT_RIGHT_TRANSFORM, intrinsics);
    backLeft = new Camera(field, BACK_LEFT_NAME, BACK_LEFT_TRANSFORM, intrinsics);

    sim = new VisionSim();
    sim.addCamera(frontLeft);
    sim.addCamera(frontRight);
    sim.addCamera(backLeft);
  }

  public boolean feedPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
    boolean tfl = frontLeft.feedPoseEstimator(poseEstimator);
    boolean tfr = frontRight.feedPoseEstimator(poseEstimator);
    boolean tbl = backLeft.feedPoseEstimator(poseEstimator);
    return tfl || tfr || tbl;
    // return tfr;
  }

  @Override
  public void periodic() {
    sim.periodic(swerve.getPoseWheelsOnly());
    frontLeft.logCamTransform(swerve.getPose());
    frontRight.logCamTransform(swerve.getPose());
    backLeft.logCamTransform(swerve.getPose());
  }
}
