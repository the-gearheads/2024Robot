package frc.robot.subsystems;


import java.util.Map;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static java.util.Map.entry;

public class ShooterCalculations {
  // not quite sure whether to have separate variables for red and blue but for now this is fine
  static Translation3d speakerPosition = new Translation3d(0.173, 5.543, 2.216);
  // Distance (m) -> Angle (rad)
  static Map<Double, Double> shooterMap = Map.ofEntries(
    entry(0.0, 1.57),
    entry(2.0, 0.5),
    entry(4.0, 0.0)
  );
  static PolynomialSplineFunction shooterAngleFunction = new SplineInterpolator().interpolate(shooterMap.keySet().stream().mapToDouble(d -> d).toArray(),
                                                                                              shooterMap.values().stream().mapToDouble(d -> d).toArray());
  
  public static Rotation2d getYawToSpeaker(Translation2d robotPos) {
    Translation2d xyPos = speakerPosition.toTranslation2d();
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    if(isRed) {
      xyPos = GeometryUtil.flipFieldPosition(xyPos);
    }
    Rotation2d angle = new Rotation2d(Math.atan2(xyPos.getY() - robotPos.getY(), xyPos.getX() - robotPos.getX()));

    if (isRed) {
      angle = angle.plus(new Rotation2d(Math.PI));
    }

    return angle;
  }

  public static double getShooterDistance(Translation2d robotPos) {
    return speakerPosition.toTranslation2d().getDistance(robotPos);
  }

  public static double getShooterAngle(Translation2d robotPos) {
    return shooterAngleFunction.value(getShooterDistance(robotPos));
  }
}
