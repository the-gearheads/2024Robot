package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ScoringState;
import frc.robot.controllers.Controllers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.ScoringState.GoalMode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

// Not the biggest fan of this but we legit need like 80 different constants from each subclass
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterCalculations {
  // not quite sure whether to have separate variables for red and blue but for now this is fine
  static final Translation3d speakerPosition = new Translation3d(0.173, 5.543, 2.05);
  // static Translation3d speakerPosition = new Translation3d(0.173, 5.543, 1.9);
  static final Translation2d speakerBackPosition = new Translation2d(0.0, 5.55);  // 0.127 offset bc notes arc left in shooter
  static final Translation2d ampPosition = new Translation2d(1.85, 8.15);
  static final Translation2d stageCenter = new Translation2d(4.89, 4.09);

  static Translation2d feedPosition = new Translation2d(2.01, 6.05);
  static Translation2d outsideWingFeedPosition = new Translation2d(6.8, 6.8);
  // Distance (m) -> Angle (rad)
  static final PolynomialSplineFunction shooterAngleFunction = new SplineInterpolator().interpolate(SPLINE_DISTANCES, SPLINE_ANGLES);
  
  private static boolean isOnRed() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static boolean isInSource(Translation2d robotPos) {
    var sourceCenter = SOURCE_CENTER;
    if(isOnRed()) {
      sourceCenter = GeometryUtil.flipFieldPosition(sourceCenter);
    }
    return robotPos.getDistance(sourceCenter) < SOURCE_RADIUS;
  }

  private static double getEstimatedShotDuration(double distance) {
    double shotSpeed = 20; // replace this with an interpolation table or something i just took this value from someone else's code from memory
    return distance / shotSpeed;
  }

  /* Offset the point we're aiming for by our velocity in the opposite direction */
  private static Translation2d getVelAdjustedSpeakerPoint(Translation2d robotPos, ChassisSpeeds fieldRelativeRobotSpeeds) {
    double shotDur = getEstimatedShotDuration(getDistanceToSpeaker(robotPos, speakerBackPosition));
    double x = speakerBackPosition.getX() - fieldRelativeRobotSpeeds.vxMetersPerSecond * shotDur;
    double y = speakerBackPosition.getY() - fieldRelativeRobotSpeeds.vyMetersPerSecond * shotDur;
    Logger.recordOutput("Calculations/AdjustedSpeakerPos", new Pose2d(x, y, new Rotation2d()));
    return new Translation2d(x, y);
  }

  /* Gets yaw specifically for speaker */
  private static Rotation2d getYawSpeaker(Translation2d robotPos, ChassisSpeeds robotSpeeds) {
    Translation2d target = getVelAdjustedSpeakerPoint(robotPos, robotSpeeds);
    if(isOnRed()) {
      target = GeometryUtil.flipFieldPosition(speakerBackPosition);
    }

    Logger.recordOutput("Calculations/SpeakerBackPos", new Pose3d(new Translation3d(target.getX(), target.getY(), speakerPosition.getZ()), new Rotation3d()));
    Rotation2d angle = target.minus(robotPos).getAngle();

    Logger.recordOutput("Calculations/YawToSpeaker", angle.getDegrees());

    return angle;
  }

  /* Gets yaw specifically for feeding */
  private static Rotation2d getYawFeed(Translation2d robotPos) {
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d targetAngle = getFeedPosition(robotPos);
    if(isRed) {
      targetAngle = GeometryUtil.flipFieldPosition(feedPosition);
    }

    Rotation2d angle = targetAngle.minus(robotPos).getAngle();
    Logger.recordOutput("Calculations/YawToFeed", angle.getDegrees());

    return angle;
  }

  public static double getFeedRpm(Translation2d robotPos) {
    Translation2d targetTrans = getFeedPosition(robotPos);
    double distToFeedPoint = targetTrans.getDistance(robotPos);
    Logger.recordOutput("Calculations/DistToFeedPoint", distToFeedPoint);

    distToFeedPoint = MathUtil.clamp(distToFeedPoint, FEED_SPEED_INTERP_DISTS[0], FEED_SPEED_INTERP_DISTS[FEED_SPEED_INTERP_DISTS.length-1]);

    double rpm = FEED_SPEED_INTERP.value(distToFeedPoint);
    Logger.recordOutput("Calculations/CalculatedShooterRPM", rpm);
    return rpm;
  }

  private static Translation2d getFeedPosition(Translation2d robotPos) {
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d targetAngle = SOURCE_CENTER;
    if(isRed) {
      targetAngle = GeometryUtil.flipFieldPosition(SOURCE_CENTER);
    }

    double distToSource = targetAngle.getDistance(robotPos);
    Logger.recordOutput("Calculations/DistToSource", distToSource);
    if (distToSource < SOURCE_WING_DIST) {
      return isRed ? GeometryUtil.flipFieldPosition(outsideWingFeedPosition) : outsideWingFeedPosition;
    } else {
      return isRed ? GeometryUtil.flipFieldPosition(feedPosition) : feedPosition;
    }
  }

  /* Gets yaw specifically for stage align (snaps to closest stage angle) */
  private static Rotation2d getYawStage(Translation2d robotPos) {
    Translation2d targetAngle = stageCenter;
    Rotation2d stage1Snap = STAGE_1_SNAP;
    Rotation2d stage2Snap = STAGE_2_SNAP;
    Rotation2d stageCenterSnap = STAGE_CENTER_SNAP;
    if(isOnRed()) {
      targetAngle = GeometryUtil.flipFieldPosition(targetAngle);
      stage1Snap = GeometryUtil.flipFieldRotation(stage1Snap);
      stage2Snap = GeometryUtil.flipFieldRotation(stage2Snap);
      stageCenterSnap = GeometryUtil.flipFieldRotation(stageCenterSnap);
    }

    Rotation2d angle = targetAngle.minus(robotPos).getAngle();
    Logger.recordOutput("Calculations/YawToStage", angle.getDegrees());
    double diff1 = Math.abs(angle.minus(stage1Snap).getRadians());
    double diff2 = Math.abs(angle.minus(stage2Snap).getRadians());
    double diffCenter = Math.abs(angle.minus(stageCenterSnap).getRadians());

    double minDiff = Math.min(diff1, Math.min(diff2, diffCenter));
    Rotation2d closestAngle;
    if (minDiff == diff1)
        closestAngle = stage1Snap;
    else if (minDiff == diff2)
        closestAngle = stage2Snap;
    else
        closestAngle = stageCenterSnap;

    
    Logger.recordOutput("Calculations/YawToStageSnapped", closestAngle.getDegrees());
    return closestAngle;
  }

  public static double getDistanceToSpeaker(Translation2d robotPos, Translation2d speakerPos) {
    Translation2d pos = speakerPos;
    if(isOnRed()) {
      pos = GeometryUtil.flipFieldPosition(pos);
    }
    Logger.recordOutput("Calculations/SpeakerPos", new Pose3d(new Translation3d(pos.getX(), pos.getY(), speakerPosition.getZ()), new Rotation3d()));
    double dist = pos.getDistance(robotPos);
    Logger.recordOutput("Calculations/DistanceToSpeaker", dist);
    return dist;
  }

  public static double getDistanceToSpeaker(Swerve swerve) {
    return getDistanceToSpeaker(swerve.getPose().getTranslation(), getVelAdjustedSpeakerPoint(swerve.getPose().getTranslation(), swerve.getFieldRelativeSpeeds()));
  }

  public static double getDistanceToAmp(Translation2d robotPos) {
    Translation2d pos = ampPosition;
    if(isOnRed()) {
      pos = GeometryUtil.flipFieldPosition(pos);
    }
    Logger.recordOutput("Calculations/AmpPose", new Pose2d(ampPosition, new Rotation2d()));
    double dist = pos.getDistance(robotPos);
    Logger.recordOutput("Calculations/DistanceToAmp", dist);
    return dist;
  }



  private static double getShooterAngleSpeaker(Translation2d robotPos, ChassisSpeeds robotVel) {
    double distance = getDistanceToSpeaker(robotPos, getVelAdjustedSpeakerPoint(robotPos, robotVel));
    
    if(distance > SPLINE_DISTANCES[SPLINE_DISTANCES.length - 1]) {
      return SPLINE_ANGLES[SPLINE_ANGLES.length - 1];
    }

    if(distance < SPLINE_DISTANCES[0]) {
      return SPLINE_ANGLES[0];
    }

    double angle = shooterAngleFunction.value(distance);
    Logger.recordOutput("Calculations/ShooterAngle", angle);
    return angle;
  }
  
  public static double getShooterAngle(Translation2d robotPos, ChassisSpeeds robotFieldSpeeds, boolean wantToShoot) {
    switch(ScoringState.goalMode) {
      case SPEAKER:
        return getShooterAngleSpeaker(robotPos, robotFieldSpeeds);
      case AMP:
        if(wantToShoot) {
          return ShooterConstants.AMP_SCORE_ANGLE;
        }
        if (getDistanceToAmp(robotPos) < 1) {
          return ShooterConstants.AMP_WAIT_ANGLE;
        }
        return ShooterConstants.STOW_ANGLE;
      case STAGE:
        return ShooterConstants.STOW_ANGLE;
      default:
        return ShooterConstants.STOW_ANGLE;
    }
  }

  public static double getShooterAngle(Swerve swerve) {
    return getShooterAngle(swerve.getPose().getTranslation(), swerve.getFieldRelativeSpeeds(), false);
  }

  public static double getShooterAngle(Swerve swerve, boolean wantToShoot) {
    return getShooterAngle(swerve.getPose().getTranslation(), swerve.getFieldRelativeSpeeds(), wantToShoot);
  }

  public static double getShooterAngle(Translation2d robotPos, ChassisSpeeds robotFieldSpeeds) {
    return getShooterAngle(robotPos, robotFieldSpeeds, false);
  }

  public static Rotation2d getYaw(Swerve swerve) {
    Translation2d robotPos = swerve.getPose().getTranslation();
    ChassisSpeeds robotVel = swerve.getFieldRelativeSpeeds();
    if(Controllers.driverController.getAimAndFeedBtn().getAsBoolean()) {
      return getYawFeed(robotPos);
    }

    var sourceYaw = new Rotation2d(SOURCE_YAW);
    var sourceBabyBirdYaw = new Rotation2d(BABY_BIRD_YAW);
    if(isOnRed()) {
      sourceYaw = GeometryUtil.flipFieldRotation(sourceYaw);
      sourceBabyBirdYaw = GeometryUtil.flipFieldRotation(sourceBabyBirdYaw);
    }

    // Baby Bird mode (feed directly from source)
    if(isInSource(robotPos) && DriverStation.isTeleopEnabled() && ScoringState.goalMode != GoalMode.STAGE) {
        return ScoringState.babyBirdMode ? sourceBabyBirdYaw : sourceYaw;
    }

    switch(ScoringState.goalMode) {
      default:
      case SPEAKER:
        return getYawSpeaker(robotPos, robotVel);
        // return getYawSpeaker(robotPos);
      case AMP:
        return new Rotation2d(AMP_YAW);
      case STAGE:
        return getYawStage(robotPos);
    }
  }

  public static void setShooterPower(Shooter shooter) {
    switch(ScoringState.goalMode) {
      case AMP:
        shooter.setTopSpeed(-AMP_SPEED);
        shooter.setBottomSpeed(AMP_SPEED);
        break;
      case SPEAKER:
        shooter.setSpeed(DEFAULT_SPEED);
        break;
      case STAGE:
        shooter.setSpeed(0);
        // arm.setAngle(ArmConstants.MIN_ANGLE);
        break;
    }
  }




  public static double getYawTolerance(Translation2d robotPos) {
    switch(ScoringState.goalMode) {
      case SPEAKER:
        double dist = getDistanceToSpeaker(robotPos, speakerBackPosition);
        if (yawToleranceInterpolationTable.isValidPoint(dist)) return yawToleranceInterpolationTable.value(dist);
        if (dist < SHOOT_YAW_TOLERANCE_DISTS[0]) return SHOOT_YAW_TOLERANCE_YAWS[0]; 
        return SHOOT_YAW_TOLERANCE_YAWS[SHOOT_YAW_TOLERANCE_YAWS.length - 1]; 
      default:
      case AMP:
        return FACING_AMP_TOLERANCE;
      case STAGE:
        return FACING_STAGE_TOLERANCE;
    }
  }

  public static double getArmTolerance(Translation2d robotPos) {
    switch(ScoringState.goalMode) {
      case SPEAKER:
        double dist = getDistanceToSpeaker(robotPos, speakerBackPosition);
        if (armToleranceInterpolationTable.isValidPoint(dist)) return armToleranceInterpolationTable.value(dist);
        if (dist < SHOOTING_ARM_TOLERANCES_DISTS[0]) return SHOOTING_ARM_TOLERANCES_ANGLES[0]; 
        return SHOOTING_ARM_TOLERANCES_ANGLES[SHOOTING_ARM_TOLERANCES_ANGLES.length - 1]; 
      default:
      case AMP:
        return AMP_ANGLE_TOLERANCE;
      case STAGE:
        return AMP_ANGLE_TOLERANCE;
    }
  }
}
