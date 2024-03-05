package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ScoringState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ShooterConstants.AMP_SPEED;
import static frc.robot.Constants.ShooterConstants.DEFAULT_SPEED;
import static frc.robot.Constants.SwerveConstants.AMP_YAW;

public class ShooterCalculations {
  // not quite sure whether to have separate variables for red and blue but for now this is fine
  static Translation3d speakerPosition = new Translation3d(0.173, 5.543, 2.216);
  static Translation2d speakerBackPosition = new Translation2d(0.0, 5.55);
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

  public static double getShooterAngle(Translation2d robotPos) {
    switch(ScoringState.goalMode) {
      case SPEAKER:
        return getShooterAngleSpeaker(robotPos);
      case AMP:
      default:
        return ShooterConstants.AMP_ANGLE;
    }
  }

  public static Rotation2d getYaw(Translation2d robotPos) {
    switch(ScoringState.goalMode) {
      case SPEAKER:
        return getYawSpeaker(robotPos);
      case AMP:
      default:
        return new Rotation2d(AMP_YAW);
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
        // shooter.setSpeed(DEFAULT_SPEED);
        // arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
        break;
    }
  }
}
