package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.Command;
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
    gyro.zeroYaw();
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

    resetPose(new Pose2d(new Translation2d(2, 2), new Rotation2d()));

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.7, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.7, 0.0, 0.0), // Rotation PID constants
                MAX_MOD_SPEED, // Max module speed, in m/s
                WHEEL_POSITIONS[0].getX(), // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE S
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public Rotation2d getGyroRotation() {
    /* gyro's inverted i believe */
    return Rotation2d.fromRadians(gyro.getRotation2d().getRadians());
  }

  public Command pathFindTo(Pose2d targetPose) {
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            Constants.AutoConstants.PATHFIND_CONSTRAINTS,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    return pathfindingCommand;

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
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));
  }

  public SwerveModulePosition[] getModulePositions() {
    getModuleStates();
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    Logger.recordOutput("Swerve/Positions", positions);
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    Logger.recordOutput("Swerve/States", states);
    return states;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
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
    // var states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 1));
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDriveVolts(v);
      // states[i].speedMetersPerSecond = v;
      // modules[i].setStateVoltage(states[i]);
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
