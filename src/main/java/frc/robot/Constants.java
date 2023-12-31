// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
  public static class SwerveConstants {
    // drive, then turn motor ids
    public static final int[][] MOTOR_IDS = {
      {1, 2}, // FL
      {3, 4}, // FR
      {5, 6}, // BL
      {7, 8}  // BR
    };

    // Again, FL, FR, BL, BR
    public static final Translation2d[] WHEEL_POSITIONS = {
      new Translation2d(0.311, 0.337), // placeholders, note that +x is forward and +y is left
      new Translation2d(0.311, -0.337),
      new Translation2d(-0.311, 0.337),
      new Translation2d(-0.311, -0.337)
    };

    public static final double[] WHEEL_OFFSETS = {270, 0, 180, 90};

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    public static final double DRIVE_PINION_TOOTH_COUNT = 14;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVE_RATIO = (45.0 * 22) / (DRIVE_PINION_TOOTH_COUNT * 15);

    // Throughbore encoder is directly on the output steer shaft
    public static final double STEER_RATIO = 1;

    public static final double DRIVE_POS_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_RATIO; // rotations -> gear ratio adjusted rotations -> meters
    public static final double DRIVE_VEL_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_RATIO / 60.0; // rpm -> gear ratio adjusted rpm -> meters/min -> meters/sec 

    public static final double DRIVE_FREE_RPM = 5676;
    public static final double DRIVE_FREE_SPD = DRIVE_FREE_RPM * DRIVE_VEL_FACTOR; // Convert max neo free speed to max free wheel speed

    public static final double STEER_POS_FACTOR = 2 * Math.PI; // rotations -> radians
    public static final double STEER_VEL_FACTOR = 2 * Math.PI * 60; // rpm -> rad/sec


    public static final double[] STEER_PIDF = {1, 0, 0, 0}; // apparently just a P value of 1 worked for us??? i wanna test that a bit more throughly
    public static final double[] DRIVE_PIDF = {0.04, 0, 0, 1 / DRIVE_FREE_SPD};

    // For desaturateWheelSpeeds
    public static final double MAX_MOD_SPEED = 1;  // m/s, placeholders
    public static final double MAX_MOD_TRANS_SPEED = 1; // m/s
    public static final double MAX_MOD_ROT_SPEED = 1; // rad/s
  }

  public static class Controllers {
    public static final double JOYSTICK_DEADBAND = 0.05;

    public static final double BASE_TRANS_SPEED = 4;
    public static final double BASE_ROT_SPEED = 2;
    public static final double MOD_TRANS_SPEED_FACTOR = 5;
    public static final double MOD_ROT_SPEED_FACTOR = 3;

  }
}
