package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.Constants.FieldConstants.*;

public class RobotInZone {
  Pose2d pose;
  Polygon stage = new Polygon(STAGE_X, STAGE_Y);
  Polygon wing = new Polygon(WING_X, WING_Y);
  Polygon shooterSpin = new Polygon(SHOOTER_SPIN_X, SHOOTER_SPIN_Y);

  public RobotInZone(Pose2d pose) {
    this.pose = pose;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  public boolean robotInStage() {
    return stage.contains(pose);
  }
  
  public boolean robotInWing() {
    return wing.contains(pose);
  }
  
  public boolean robotInShooterSpin() {
    return shooterSpin.contains(pose);
  }


}