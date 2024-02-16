package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShooterCalculations {
  // not quite sure whether to have separate variables for red and blue but for now this is fine
  static Translation3d speakerPosition = new Translation3d(0.173, 5.543, 2.216);
  // Distance (m) -> Angle (rad)
  static final double[] distances = { 0.0,  3.2,  4.0};
  static final double[] angles =    { 1.2, 0.8,  0.0};
  static PolynomialSplineFunction shooterAngleFunction = new SplineInterpolator().interpolate(distances, angles);
  
  public static Rotation2d getYawToSpeaker(Translation2d robotPos) {
    Translation2d xyPos = speakerPosition.toTranslation2d();
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    if(isRed) {
      xyPos = GeometryUtil.flipFieldPosition(xyPos);
    }
    Rotation2d angle = new Rotation2d(Math.atan2(xyPos.getY() - robotPos.getY(), xyPos.getX() - robotPos.getX()));

    if (!isRed) {
      angle = angle.plus(new Rotation2d(Math.PI));
    }

    Logger.recordOutput("Calculations/YawToSpeaker", angle.getDegrees());

    return angle;
  }

  public static double getDistanceToSpeaker(Translation2d robotPos) {
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d pos = speakerPosition.toTranslation2d();
    if(isRed) {
      pos = GeometryUtil.flipFieldPosition(pos);
    }
    return pos.getDistance(robotPos);
  }

  /* Need to account for stage and other things in the future */
  public static double getShooterAngle(Translation2d robotPos) {
    double distance = getDistanceToSpeaker(robotPos);
    if(distance > distances[distances.length - 1]) {
      return angles[angles.length - 1];
    }

    if(distance < distances[0]) {
      return angles[0];
    }

    double angle = shooterAngleFunction.value(distance);
    Logger.recordOutput("Calculations/ShooterAngle", angle);
    return angle;
  }
}
