package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ScoringState;
import frc.robot.controllers.Controllers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

// Not the biggest fan of this but we legit need like 80 different constants from each subclass
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterCalculations {
  // not quite sure whether to have separate variables for red and blue but for now this is fine
  static Translation3d speakerPosition = new Translation3d(0.173, 5.543, 2.05);
  // static Translation3d speakerPosition = new Translation3d(0.173, 5.543, 1.9);
  static Translation2d speakerBackPosition = new Translation2d(0.0, 5.55);  // 0.127 offset bc notes arc left in shooter
  static Translation2d ampPosition = new Translation2d(1.85, 8.15);
  static Translation2d stageCenter = new Translation2d(4.89, 4.09);
  // Distance (m) -> Angle (rad)
  static PolynomialSplineFunction shooterAngleFunction = new SplineInterpolator().interpolate(SPLINE_DISTANCES, SPLINE_ANGLES);
  
  private static Rotation2d getYawSpeaker(Translation2d robotPos) {
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d targetAngle = speakerBackPosition;
    if(isRed) {
      targetAngle = GeometryUtil.flipFieldPosition(speakerBackPosition);
    }

    Logger.recordOutput("Calculations/SpeakerBackPos", new Pose3d(new Translation3d(targetAngle.getX(), targetAngle.getY(), speakerPosition.getZ()), new Rotation3d()));

    // Rotation2d angle = new Rotation2d(Math.atan2(xyPos.getY() - robotPos.getY(), xyPos.getX() - robotPos.getX()));
    Rotation2d angle = targetAngle.minus(robotPos).getAngle();


    Logger.recordOutput("Calculations/YawToSpeaker", angle.getDegrees());

    return angle;
  }

  private static Rotation2d getYawStage(Translation2d robotPos) {
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d targetAngle = stageCenter;
    Rotation2d stage1Snap = STAGE_1_SNAP;
    Rotation2d stage2Snap = STAGE_2_SNAP;
    Rotation2d stageCenterSnap = STAGE_CENTER_SNAP;
    if(isRed) {
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

  public static double getDistanceToSpeaker(Translation2d robotPos) {
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d pos = speakerPosition.toTranslation2d();
    if(isRed) {
      pos = GeometryUtil.flipFieldPosition(pos);
    }
    Logger.recordOutput("Calculations/SpeakerPos", new Pose3d(speakerPosition, new Rotation3d()));
    double dist = pos.getDistance(robotPos);
    Logger.recordOutput("Calculations/DistanceToSpeaker", dist);
    return dist;
  }

  public static double getDistanceToAmp(Translation2d robotPos) {
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d pos = ampPosition;
    if(isRed) {
      pos = GeometryUtil.flipFieldPosition(pos);
    }
    Logger.recordOutput("Calculations/AmpPose", new Pose2d(ampPosition, new Rotation2d()));
    double dist = pos.getDistance(robotPos);
    Logger.recordOutput("Calculations/DistanceToAmp", dist);
    return dist;
  }

  /* Need to account for stage and other things in the future */
  private static double getShooterAngleSpeaker(Translation2d robotPos) {
    double distance = getDistanceToSpeaker(robotPos);
    
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
  
  /* Need to account for stage and other things in the future */
  private static double getMathShooterAngleSpeaker(Translation2d robotPos) {
    double distance = getDistanceToSpeaker(robotPos);
    double pivotToSpeakerHeight = speakerPosition.getZ() - SHOOTER_PIVOT_HEIGHT;
    Logger.recordOutput("Calculations/pivotToSpeakerHeight", pivotToSpeakerHeight);
    double angle = Math.atan2(pivotToSpeakerHeight, distance);
    Logger.recordOutput("Calculations/atan2ShooterAngle", angle);
    return angle;
  }
  
  public static double getYawTolerance(Translation2d robotPos) {
    switch(ScoringState.goalMode) {
      case SPEAKER:
        double dist = getDistanceToSpeaker(robotPos);
        if (yawToleranceInterpolationTable.isValidPoint(dist)) return yawToleranceInterpolationTable.value(dist);
        if (dist < SHOOT_YAW_TOLERANCE_DISTS[0]) return SHOOT_YAW_TOLERANCE_YAWS[0]; 
        return SHOOT_YAW_TOLERANCE_YAWS[SHOOT_YAW_TOLERANCE_YAWS.length - 1]; 
      case AMP:
        return FACING_AMP_TOLERANCE;
      case STAGE:
        return FACING_STAGE_TOLERANCE;
      default:
        return 0.01;
    }
  }

  public static double getArmTolerance(Translation2d robotPos) {
    switch(ScoringState.goalMode) {
      case SPEAKER:
        double dist = getDistanceToSpeaker(robotPos);
        if (armToleranceInterpolationTable.isValidPoint(dist)) return armToleranceInterpolationTable.value(dist);
        if (dist < SHOOTING_ARM_TOLERANCES_DISTS[0]) return SHOOTING_ARM_TOLERANCES_ANGLES[0]; 
        return SHOOTING_ARM_TOLERANCES_ANGLES[SHOOTING_ARM_TOLERANCES_ANGLES.length - 1]; 
      case AMP:
        return AMP_ANGLE_TOLERANCE;
      case STAGE:
        return AMP_ANGLE_TOLERANCE;
      default:
        return 0.01;
    }
  }
  public static double getShooterAngle(Translation2d robotPos, boolean wantToShoot) {
    switch(ScoringState.goalMode) {
      case SPEAKER:
      getMathShooterAngleSpeaker(robotPos);
      return getShooterAngleSpeaker(robotPos);
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

  public static double getShooterAngle(Translation2d robotPos) {
    return getShooterAngle(robotPos, false);
  }

  public static Rotation2d getYaw(Translation2d robotPos) {
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    if(Controllers.operatorController.getBabyBird().getAsBoolean()) { // Baby Bird mode (feed directly from source)
      var yaw = new Rotation2d(BABY_BIRD_YAW);
      if(isRed) {
        return GeometryUtil.flipFieldRotation(yaw);
      }
      return yaw;
    }

    var sourceCenter = SOURCE_CENTER;
    var sourceYaw = new Rotation2d(SOURCE_YAW);
    if(isRed) {
      sourceCenter = GeometryUtil.flipFieldPosition(sourceCenter);
      sourceYaw = GeometryUtil.flipFieldRotation(sourceYaw);
    }

    if(robotPos.getDistance(sourceCenter) < SOURCE_RADIUS) {
      return sourceYaw;
    }

    switch(ScoringState.goalMode) {
      default:
      case SPEAKER:
        return getYawSpeaker(robotPos);
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
}
