package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.*;

public class Swerve extends SubsystemBase {
  AHRS gyro = new AHRS();
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(kWheelPositions);
  SwerveDriveOdometry odometry;
  SwerveModule[] modules = {
    new SwerveModule(kMotorIds[0][0], kMotorIds[0][1], kWheelOffsets[0], "FL"),
    new SwerveModule(kMotorIds[1][0], kMotorIds[1][1], kWheelOffsets[1], "FR"),
    new SwerveModule(kMotorIds[2][0], kMotorIds[2][1], kWheelOffsets[2], "BL"),
    new SwerveModule(kMotorIds[3][0], kMotorIds[3][1], kWheelOffsets[3], "BR")
  };

  public Swerve() {
    odometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), getModuleStates());
  }

  public Rotation2d getGyroRotation() {
    /* gyro's inverted i believe */
    return Rotation2d.fromRadians(-gyro.getRotation2d().getRadians());
  }

  public void drive(ChassisSpeeds speeds) {
    ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discretized);

    if (SmartDashboard.getBoolean("/Swerve/desaturateWheelSpeeds", true)) {
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, discretized, kMaxModuleSpeed, kMaxModuleTranslationalSpeed, kMaxModuleRotationalVelocity);
    }

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(moduleStates[i]);
    }
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroRotation()));
  }

  public SwerveModulePosition[] getModuleStates() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getState();
    }
    return positions;
  }

  @Override
  public void periodic() {
    odometry.update(getGyroRotation(), getModuleStates());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public void resetPose(Pose2d pose) {
    for (SwerveModule module : modules) {
      module.resetEncoders();
    }
    odometry.resetPosition(getGyroRotation(), getModuleStates(), pose);
  }
}
