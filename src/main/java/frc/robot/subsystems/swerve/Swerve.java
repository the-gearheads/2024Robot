package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import static frc.robot.Constants.SwerveConstants.*;

public class Swerve {
  AHRS gyro = new AHRS();
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(kWheelPositions);
  SwerveDriveOdometry odometry;

  public Swerve() {
    gyro.reset();


    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new SwerveModulePosition[4]);
  }
}
