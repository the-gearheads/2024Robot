// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Polygon;


import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final long THREAD_SLEEP_TIME = 200; // probably enough, really just only speeds up startup
  public static class SwerveConstants {
    // drive, then turn motor ids
    public static final int[][] MOTOR_IDS = {
      {6, 26}, // FL
      {8, 28}, // FR
      {2, 22}, // BL
      {4, 24}  // BR
    };

    // Again, FL, FR, BL, BR
    public static final Translation2d[] WHEEL_POSITIONS = {
      new Translation2d(0.3305, 0.3313), // placeholders, note that +x is forward and +y is left
      new Translation2d(0.3305, -0.3313),
      new Translation2d(-0.3305, 0.3313),
      new Translation2d(-0.3305, -0.3313)
    };
    public static final double MODULE_RADIUS = WHEEL_POSITIONS[0].getNorm();

    public static final double[] WHEEL_OFFSETS = {90, 0, 0, 90}; // could be wrong, we get to find out {270, 0, 180, 90};
    public static final boolean[] IS_INVERTED = {true, false, true, false}; // left side inverted i think

    public static final double WHEEL_DIAMETER = 0.073800;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    public static final double DRIVE_PINION_TOOTH_COUNT = 14;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVE_RATIO = (45.0 * 22) / (DRIVE_PINION_TOOTH_COUNT * 15);

    public static final double ODOMETRY_FREQUENCY = 50;

    // Throughbore encoder is directly on the output steer shaft
    public static final double STEER_RATIO = 1;

    public static final double DRIVE_POS_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_RATIO; // rotations -> gear ratio adjusted rotations -> meters
    public static final double DRIVE_VEL_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_RATIO / 60.0; // rpm -> gear ratio adjusted rpm -> meters/min -> meters/sec 

    public static final double DRIVE_FREE_RPM = 6784;
    public static final double DRIVE_FREE_SPD = DRIVE_FREE_RPM * DRIVE_VEL_FACTOR; // Convert max neo free speed to max free wheel speed

    public static final double STEER_POS_FACTOR = 2 * Math.PI; // rotations -> radians
    public static final double STEER_VEL_FACTOR = (2 * Math.PI) / 60.0; // rpm -> rad/sec


    public static final double[] STEER_PIDF = {1, 0, 0, 0}; // apparently just a P value of 1 worked for us??? i wanna test that a bit more throughly
    public static final double[] DRIVE_PID = {0.04, 0, 0};
    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.25521, 2.0821, 0.10605);


    public static final int DRIVE_CURRENT_LIMIT = 50;
    public static final int STEER_CURRENT_LIMIT = 20;

    public static final double PATHPLANNER_MAX_MOD_SPEED = 5; // m/s I think
    // For desaturateWheelSpeeds 
    public static final boolean DESATURATE = true;
    public static final double MAX_MOD_SPEED = DRIVE_FREE_SPD;  // m/s, placeholders
    public static final double MAX_ROBOT_TRANS_SPEED = DRIVE_FEEDFORWARD.maxAchievableVelocity(12, 0.1); // m/s
    public static final double MAX_ROBOT_ROT_SPEED = MAX_ROBOT_TRANS_SPEED / 0.4585738763; // rad/s, 0.45 is radius of robot, spd/r is rad/s

    
    public static final double AMP_YAW = (270d / 360d) * 2 * Math.PI;
    public static final double BABY_BIRD_YAW = (300d / 360d) * 2 * Math.PI; // 60 degrees in -some- direction, idk which but i think it's this one
    public static final double SOURCE_YAW = (120d / 360d) * 2 * Math.PI;

    public static final double FACING_SPEAKER_TOLERANCE = (2.5 / 360.0) * (2 * Math.PI);
    public static final double FACING_AMP_TOLERANCE = (10
    
     / 360.0) * (2 * Math.PI);
    public static final double FACING_STAGE_TOLERANCE = (1 / 360.0) * (2 * Math.PI);
    public static final double[] SHOOT_YAW_TOLERANCE_DISTS = {1.5, 2, 3};
    public static final double[] SHOOT_YAW_TOLERANCE_YAWS = {0.1, 0.07, 0.044};
    public static final PolynomialSplineFunction yawToleranceInterpolationTable = new LinearInterpolator().interpolate(SHOOT_YAW_TOLERANCE_DISTS, SHOOT_YAW_TOLERANCE_YAWS);

    public static final double NOTE_FEEDING_YAW_TOLERANCE = (6 / 360.0) * (2 * Math.PI);;
  }

  public static class AutoConstants {
    public static final PathConstraints PATHFIND_CONSTRAINTS = new PathConstraints(
            0.2, 0.5,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
  }

  public static class VisionConstants {
    public static final String NOTE_CAM_NAME = "Microsoft_LifeCam_HD-3000";
    public static final String FRONT_RIGHT_NAME = "FRONT_RIGHT";
    public static final String FRONT_LEFT_NAME = "FRONT_LEFT";
    public static final String BACK_LEFT_NAME = "BACK_LEFT";
    

    public static final Transform3d NOTE_CAM_TRANSFORM = 
      new Transform3d(
        new Translation3d(-0.35433, -0.2667, 0.2286 + 0.051), // 0.0254 1 inches (1/2 note height)
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180 - 9))
    );
    // Front Left, Front Right all measured from cad, except for pitch roll and yaw.
    public static final Transform3d FRONT_LEFT_TRANSFORM = 
      new Transform3d(new Translation3d(0.3509899, 0.1526159, 0.2167636),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-21), Units.degreesToRadians(-31.0))
    );

    public static final Transform3d FRONT_RIGHT_TRANSFORM = new Transform3d(
      new Translation3d(0.3509899, -0.15261844, 0.21686012),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-25.5), Units.degreesToRadians(29.0))
    );
    // Front Left Front Right, measured IRl to high precicsey
    // public static final Transform3d FRONT_LEFT_TRANSFORM = 
    //   new Transform3d(new Translation3d(0.3631692, 0.225806, 0.2174875),
    //   new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-21), Units.degreesToRadians(-31.0))
    // );

    // public static final Transform3d FRONT_RIGHT_TRANSFORM = new Transform3d(
    //   new Translation3d(0.3631184, -0.271272, 0.2174875),
    //   new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-25.5), Units.degreesToRadians(29.0))
    // );
    // public static final Transform3d FRONT_LEFT_TRANSFORM = 
    //   new Transform3d(new Translation3d(0.3631692, 0.1526159, 0.2174875),
    //   new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-21), Units.degreesToRadians(-31.0))
    // );

    // public static final Transform3d FRONT_RIGHT_TRANSFORM = new Transform3d(
    //   new Translation3d(0.3631184, -0.15261844, 0.2174875),
    //   new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-25.5), Units.degreesToRadians(29.0))
    // );
    // measured IRL, to very much precision
    public static final Transform3d BACK_LEFT_TRANSFORM = new Transform3d(
      new Translation3d(-0.36195, 0.265049, 0.231775),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-21.9), Units.degreesToRadians(180))
    );

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.POSITIVE_INFINITY);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);

  }
  public static class Controllers {
    public static final double JOYSTICK_DEADBAND = 0.05;

    public static final double BASE_TRANS_SPEED = 2;
    public static final double BASE_ROT_SPEED = 2.5;
    public static final double MOD_TRANS_SPEED_FACTOR = 1.3;
    public static final double MOD_ROT_SPEED_FACTOR = 2.0;

    // Speed divided by this
    public static final double MOD_TRANS_SLOW_FACTOR = 2;
    public static final double MOD_ROT_SLOW_FACTOR = 2;

  }

  public static class FieldConstants {
    public static final double[] FIELD_X = {0.10, 0.10, 16.45, 16.45, 14.75, 1.80};
    public static final double[] FIELD_Y = {1.08, 8.14, 8.14, 1.08, 0.10, 0.10};
    public static final Polygon FIELD = new Polygon(FIELD_X, FIELD_Y);

    public static final double[] STAGE_X = {2.95, 5.98, 5.80};
    public static final double[] STAGE_Y = {4.09, 5.88, 2.50};
    
    public static final Pose2d AMP_SCORE_POSE = new Pose2d(new Translation2d(1.83, 7.75), Rotation2d.fromDegrees(-90));

    public static final Rotation2d STAGE_2_SNAP = Rotation2d.fromDegrees(-59.85);
    public static final Rotation2d STAGE_CENTER_SNAP = Rotation2d.fromDegrees(180);
    public static final Rotation2d STAGE_1_SNAP = Rotation2d.fromDegrees(59.85);

    public static final Translation2d SOURCE_CENTER = new Translation2d(15.70, 0.54);
    public static final double SOURCE_WING_DIST = 4;  // controls when to switch feeding position
    public static final double SOURCE_RADIUS = 3.2; // consider 1.9-1.6ish if you want it more sensitive (more towards 1.9)?

  }

  public static class Leds {
    public static final int PORT = 9;
    /* Rough estimate, refine later */
    public static final int LENGTH = 260;
  }

  public static class BrakeCoastButton {
    public static final int PORT = 4;
  }

}
