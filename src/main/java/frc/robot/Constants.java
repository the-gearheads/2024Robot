// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {

    // drive, then turn motor ids
    public static final int[][] kMotorIds = {
      {1, 2}, // FL
      {3, 4}, // FR
      {5, 6}, // BL
      {7, 8}  // BR
    };

    // Again, FL, FR, BL, BR
    public static final Translation2d[] kWheelPositions = {
      new Translation2d(0.311, 0.337), // placeholders, note that +x is forward and +y is left
      new Translation2d(0.311, -0.337),
      new Translation2d(-0.311, 0.337),
      new Translation2d(-0.311, -0.337)
    };

    public static final double[] kWheelOffsets = {270, 0, 180, 90};

    public static final double kWheelDiameter = Units.inchesToMeters(3);
    public static final double kWheelCircumference = Math.PI * kWheelDiameter;

    public static final double kDrivePinionToothCount = 14;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDriveGearRatio = (45.0 * 22) / (kDrivePinionToothCount * 15);

    // Throughbore encoder is directly on the output steer shaft
    public static final double kSteerGearRatio = 1;

    public static final double kDrivePosFactor = kWheelCircumference / kDriveGearRatio; // rotations -> gear ratio adjusted rotations -> meters
    public static final double kDriveVelFactor = kWheelCircumference / kDriveGearRatio / 60.0; // rpm -> gear ratio adjusted rpm -> meters/min -> meters/sec 

    public static final double kDriveMotorFreeRpm = 5676;
    public static final double kDriveFreeSpeed = kDriveMotorFreeRpm * kDriveVelFactor; // Convert max neo free speed to max free wheel speed

    public static final double kSteerPosFactor = 2 * Math.PI; // rotations -> radians
    public static final double kSteerVelFactor = 2 * Math.PI * 60; // rpm -> rad/sec


    public static final double[] kSteerPIDF = {1, 0, 0, 0}; // apparently just a P value of 1 worked for us??? i wanna test that a bit more throughly
    public static final double[] kDrivePIDF = {0.04, 0, 0, 1 / kDriveFreeSpeed};

    // For desaturateWheelSpeeds
    public static final double kMaxModuleSpeed = 1;  // m/s, placeholders
    public static final double kMaxModuleTranslationalSpeed = 1; // m/s
    public static final double kMaxModuleRotationalVelocity = 1; // rad/s
  }
}
