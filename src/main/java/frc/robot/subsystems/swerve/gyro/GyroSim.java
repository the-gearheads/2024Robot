// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSim {
  private Rotation2d rot;
  private double rate;

  public GyroSim() {
    this.rot = new Rotation2d();
  }

  public Rotation2d getRotation2d() {
    return rot;
  }

  public void setRotation2d(Rotation2d newRotation2d) {
    rot = newRotation2d;
  }

  public double getRate() {
    return this.rate;
  }

  public void setRate(double omegaRadiansPerSecond) {
    this.rate = omegaRadiansPerSecond;
  };


  public void zeroYaw() {
    rot = new Rotation2d();
  }

}