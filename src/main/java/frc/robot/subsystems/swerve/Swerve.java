package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.Constants.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  AHRS gyro = new AHRS();
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(WHEEL_POSITIONS);
  SwerveDriveOdometry odometry;
  Field2d field = new Field2d();
  SwerveModule[] modules;

  public Swerve() {
    if (Robot.isReal()) {
      modules = new SwerveModule[]{
        new SwerveModule(MOTOR_IDS[0][0], MOTOR_IDS[0][1], WHEEL_OFFSETS[0], "FL"),
        new SwerveModule(MOTOR_IDS[1][0], MOTOR_IDS[1][1], WHEEL_OFFSETS[1], "FR"),
        new SwerveModule(MOTOR_IDS[2][0], MOTOR_IDS[2][1], WHEEL_OFFSETS[2], "BL"),
        new SwerveModule(MOTOR_IDS[3][0], MOTOR_IDS[3][1], WHEEL_OFFSETS[3], "BR")
      };
    } else {
      modules = new SwerveModule[]{
        new SwerveModuleSim(MOTOR_IDS[0][0], MOTOR_IDS[0][1], WHEEL_OFFSETS[0], "FL"),
        new SwerveModuleSim(MOTOR_IDS[1][0], MOTOR_IDS[1][1], WHEEL_OFFSETS[1], "FR"),
        new SwerveModuleSim(MOTOR_IDS[2][0], MOTOR_IDS[2][1], WHEEL_OFFSETS[2], "BL"),
        new SwerveModuleSim(MOTOR_IDS[3][0], MOTOR_IDS[3][1], WHEEL_OFFSETS[3], "BR")
      };
    }
    odometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), getModulePositions());
    SmartDashboard.putData("Field", field);
  }

  public Rotation2d getGyroRotation() {
    /* gyro's inverted i believe */
    return Rotation2d.fromRadians(-gyro.getRotation2d().getRadians());
  }

  public void drive(ChassisSpeeds speeds) {
    ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discretized);

    if (SmartDashboard.getBoolean("/Swerve/desaturateWheelSpeeds", true)) {
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, discretized, MAX_MOD_SPEED, MAX_MOD_TRANS_SPEED, MAX_MOD_ROT_SPEED);
    }

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(moduleStates[i]);
    }
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroRotation()));
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getModulePosition();
      states[i] = modules[i].getState();
    }
    Logger.recordOutput("/Swerve/Positions", positions);
    Logger.recordOutput("/Swerve/States", states);
    return positions;
  }

  @Override
  public void periodic() {
    var t = odometry.update(getGyroRotation(), getModulePositions());

    Logger.recordOutput("/Swerve/PoseT", t);

    Logger.recordOutput("/Swerve/Pose", getPose());
    field.setRobotPose(getPose());

    for (SwerveModule module : modules) {
      module.periodic();
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public void resetPose(Pose2d pose) {
    for (SwerveModule module : modules) {
      module.resetEncoders();
    }
    odometry.resetPosition(getGyroRotation(), getModulePositions(), pose);
  }
}
