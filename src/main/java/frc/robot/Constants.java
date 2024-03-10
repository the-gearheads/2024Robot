// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import frc.robot.util.Polygon;

import static edu.wpi.first.units.Units.*;

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
      new Translation2d(0.311, 0.337), // placeholders, note that +x is forward and +y is left
      new Translation2d(0.311, -0.337),
      new Translation2d(-0.311, 0.337),
      new Translation2d(-0.311, -0.337)
    };

    public static final double[] WHEEL_OFFSETS = {90, 0, 0, 90}; // could be wrong, we get to find out {270, 0, 180, 90};
    public static final boolean[] IS_INVERTED = {true, false, true, false}; // left side inverted i think

    public static final double WHEEL_DIAMETER = 0.076;
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


    public static final int DRIVE_CURRENT_LIMIT = 80;
    public static final int STEER_CURRENT_LIMIT = 40;

    public static final double PATHPLANNER_MAX_MOD_SPEED = 5; // m/s I think
    // For desaturateWheelSpeeds 
    public static final boolean DESATURATE = true;
    public static final double MAX_MOD_SPEED = DRIVE_FREE_SPD;  // m/s, placeholders
    public static final double MAX_ROBOT_TRANS_SPEED = DRIVE_FEEDFORWARD.maxAchievableVelocity(12, 0.1); // m/s
    public static final double MAX_ROBOT_ROT_SPEED = MAX_ROBOT_TRANS_SPEED / 0.4585738763; // rad/s, 0.45 is radius of robot, spd/r is rad/s

    public static final double FACING_SPEAKER_TOLERANCE = (1.0 / 360.0) * (2 * Math.PI);

    public static final double AMP_YAW = (270d / 360d) * 2 * Math.PI;
  }

  public static class ShooterConstants {
    public static final int TOP_ID = 11;
    public static final int BOTTOM_ID = 12;
    public static final int DEFAULT_SPEED = 6000;
    public static final int AMP_SPEED = 4000;
    public static final double AMP_WAIT_ANGLE = 100d * 2d * Math.PI / 360;
    public static final double AMP_SCORE_ANGLE = 100d * 2d * Math.PI / 360;
    public static final double AMP_ANGLE_TOLERANCE = 1.2d * 2d * Math.PI / 360;
    public static final double STOW_ANGLE = 20d * 2d * Math.PI / 360; 
    public static final double SPEED_TOLERANCE = 240;
    
    public static final double[] PID = {0.0021693, 0, 0};

    public static final double SHOOTER_PIVOT_HEIGHT = 0.3048; // placeholder
    public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.045537, 0.0017932, 0.0001929);
    public static final double AUTO_SHOOTER_DISTANCE = 7;  // meters from speaker where shooter will begin spinning, in amp or speaker mode

    public static final double MAX_SHOOTING_SPEED_VX = 0.3; // m/s, preparetoshoot command waits for the chassisspeeds to be below this number before finishing
    public static final double MAX_SHOOTING_SPEED_VY = 0.3;
    public static final double MAX_SHOOTING_SPEED_ROT = 0.05; // omega rad / s
  }

  public static class ArmConstants {
    public static final int MAIN_ARM_ID = 10;
    public static final int FOLLOWER_ARM_ID = 9;
    public static final double MAX_ANGLE_DEG = 110.17978;
    public static final double MIN_ANGLE_DEG = 16.5;
    public static final double ARM_OFFSET = -0.100539;
    public static final double ARM_POS_FACTOR = 9.0/20.0 * 2 * Math.PI; // 20:9 artio between encoder and arm, also conv to radians
    public static final double ARM_ANGLE_LIVE_FF_THRESHOLD = 10; //deg
    public static final double ARM_LENGTH = 0.6660; // meters, sim and mechanism2d only
    public static final double ARM_MOTOR_GEARING = 125; // sim only, gearing between motor and arm
    public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0.19684, 0.0069965, 0.47015, 0.022602);
    public static final ArmFeedforward SIM_FEEDFORWARD = new ArmFeedforward(0.2509, 0.099081, 5.5782, 0.28261); // both of these are wrong but this one is less wrong for the purposes of sim
    public static final double[] PID = {34.566, 0, 1.0137};
    public static final Constraints ARM_CONSTRAINTS = new Constraints(
      Units.degreesToRadians(400), // max vel, deg/s  101
      Units.degreesToRadians(650) // max acc, deg/s^2 
    );
    public static final double MAX_ANGLE = (MAX_ANGLE_DEG / 360.0) * (2 * Math.PI);
    public static final double MIN_ANGLE = (MIN_ANGLE_DEG / 360.0) * (2 * Math.PI);

    // public static final double[] SPLINE_DISTANCES = {1.0959, 1.5100, 2.0153, 2.4980, 3.0284, 3.5106, 4.0048, 4.9951};  // old values
    // public static final double[] SPLINE_ANGLES =    {0.9564, 0.8569, 0.7608, 0.6672, 0.5920, 0.5588, 0.5090, 0.4762};  // old values
    public static final double[] SPLINE_DISTANCES = {1.201, 1.538, 2.026, 2.522, 3.017, 3.465, 4.00, 4.416, 5.17};
    public static final double[] SPLINE_ANGLES =    {0.893, 0.803, 0.692, 0.592, 0.507, 0.463, 0.416, 0.409, 0.380};

    public static final Measure<Voltage> armOverrideVoltage = Volts.of(4);
  }

  public static class FeederConstants {
    public static final int FEEDER_ID = 13;
    public static final int HANDOFF_ID = 7;
    public static final int IR_SWITCH_ID = 9;
    public static final int BEAMBREAK_SWITCH_ID = 7;
    public static final double[] PID = {0.00038793, 0, 0};
    public static final double[] HANDOFF_PID = {0.0019749, 0, 0}; 
    public static final double SPEED = 3000.0;
    public static final SimpleMotorFeedforward FEEDER_FF = new SimpleMotorFeedforward(0.065837, 0.0019032, 0.00021355); 
    public static final SimpleMotorFeedforward HANDOFF_FF = new SimpleMotorFeedforward(0.1132, 0.0018353, 0.00013306);
  }

  public static class IntakeConstants {
    public static final int ID = 14;
    public static final double[] PID = {0.0018087, 0, 0};
    public static final double SPEED = 5500;
    public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.26764, 0.0018386, 0.00010506);
  }

  public static class AutoConstants {
    public static final PathConstraints PATHFIND_CONSTRAINTS = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
  }

  public static class VisionConstants {
    public static final String FRONT_CAM_NAME = "front_spinel";
    public static final String BACK_CAM_NAME = "rear_spinel";
    
    public static final Transform3d FRONT_CAM_TRANSFORM = 
      new Transform3d(new Translation3d(0.28575, 0.00635, 0.212725),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-30), Units.degreesToRadians(0))
    );
    public static final Transform3d BACK_CAM_TRANSFORM = new Transform3d(
      new Translation3d(-0.3683, 0.24765, 0.2413),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-30), Units.degreesToRadians(180))
    );
  }
  public static class Controllers {
    public static final double JOYSTICK_DEADBAND = 0.05;

    public static final double BASE_TRANS_SPEED = 2;
    public static final double BASE_ROT_SPEED = 2.5;
    public static final double MOD_TRANS_SPEED_FACTOR = 2;
    public static final double MOD_ROT_SPEED_FACTOR = 2.5;

    // Speed divided by this
    public static final double MOD_TRANS_SLOW_FACTOR = 2;
    public static final double MOD_ROT_SLOW_FACTOR = 2;

  }

  public static class FieldConstants {
    public static final double[] STAGE_X = {2.95, 5.98, 5.80};
    public static final double[] STAGE_Y = {4.09, 5.88, 2.50};

    public static final double[] WING_X = {0, 0, 5.85, 5.85};
    public static final double[] WING_Y = {0, 8.15, 8.15, 0};

    public static final double[] SHOOTER_SPIN_X = {0, 0, 6.40, 6.40};
    public static final double[] SHOOTER_SPIN_Y = {0, 8.15, 8.15, 0};
    public static final Polygon SHOOTER_SPIN_ZONE = new Polygon(SHOOTER_SPIN_X, SHOOTER_SPIN_Y);

    public static final double[] AMP_UP_X = {0.65, 3.25, 2.77, 0.95};
    public static final double[] AMP_UP_Y = {8.4, 8.4, 7.52, 7.52};
    public static final Polygon AMP_UP_ZONE = new Polygon(AMP_UP_X, AMP_UP_Y);
    
    public static final Pose2d AMP_SCORE_POSE = new Pose2d(new Translation2d(1.83, 7.68), Rotation2d.fromDegrees(-90));

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
