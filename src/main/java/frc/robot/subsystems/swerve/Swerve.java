package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.util.HandledSleep;

import static frc.robot.Constants.SwerveConstants.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  AHRS gyro = new AHRS();
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(WHEEL_POSITIONS);
  SwerveDriveOdometry odometry;
  Field2d field = new Field2d();
  SwerveModule[] modules = {
    new SwerveModule(0, "FL"),
    new SwerveModule(1, "FR"),
    new SwerveModule(2, "BL"),
    new SwerveModule(3, "BR")
  };

  public Swerve() {
    odometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), getModulePositions());
    SmartDashboard.putData("Field", field);

    /* Configure the motors in batch */
    for (SwerveModule module : modules) {
      module.factoryDefaults();
    }

    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    for (SwerveModule module : modules) {
      module.configure();
    }

    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);
    for (SwerveModule module : modules) {
      module.setupStatusFrames();
    }
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    if(!DriverStation.isFMSAttached()) {
      SmartDashboard.putBoolean("Swerve/manualVoltageSteer", false);
      SmartDashboard.putBoolean("Swerve/manualVoltageDrive", false);
    }
  }

  public Rotation2d getGyroRotation() {
    /* gyro's inverted i believe */
    return Rotation2d.fromRadians(-gyro.getRotation2d().getRadians());
  }

  public void drive(ChassisSpeeds speeds) {
    ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discretized);

    if (SmartDashboard.getBoolean("Swerve/desaturateWheelSpeeds", false)) {
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
    Logger.recordOutput("Swerve/Positions", positions);
    Logger.recordOutput("Swerve/States", states);
    return positions;
  }

  @Override
  public void periodic() {
    odometry.update(getGyroRotation(), getModulePositions());
    Logger.recordOutput("Swerve/Pose", getPose());
    /* Glass doesnt support struct fields really but they're nicer to use in AScope :( */
    Logger.recordOutput("Swerve/PoseX", getPose().getX());
    Logger.recordOutput("Swerve/PoseY", getPose().getY());
    Logger.recordOutput("Swerve/PoseRotation", getPose().getRotation().getRadians());
    field.setRobotPose(getPose());

    if(!DriverStation.isFMSAttached()) {
      var steer = SmartDashboard.getBoolean("Swerve/manualVoltageSteer", false);
      var drive = SmartDashboard.getBoolean("Swerve/manualVoltageDrive", false);
      for (SwerveModule module : modules) {
        module.setSysidVoltageMode(drive, steer);
      }
    }

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

  private void sysidSetVolts(Measure<Voltage> volts) {
    double v = volts.in(Volts);
    for(var module: modules) {
      module.setDriveVolts(v);
    }
  }

  private void sysidSetVoltsSteer(Measure<Voltage> volts) {
    double v = volts.in(Volts);
    for(var module: modules) {
      module.setSteerVolts(v);
    }
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(
        null, null, null,
        (state) -> Logger.recordOutput("SysIdTestState", state.toString())
      ),
      new Mechanism(
        this::sysidSetVolts,
        null,
        this
      )
    );
  }

    public SysIdRoutine getSysIdRoutineSteer() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(
        null, null, null,
        (state) -> Logger.recordOutput("SysIdTestState", state.toString())
      ),
      new Mechanism(
        this::sysidSetVoltsSteer,
        null,
        this
      )
    );
  }
}
