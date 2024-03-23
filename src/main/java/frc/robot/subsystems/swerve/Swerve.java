package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.SwerveConstants.DESATURATE;
import static frc.robot.Constants.SwerveConstants.FACING_SPEAKER_TOLERANCE;
import static frc.robot.Constants.SwerveConstants.MAX_MOD_SPEED;
import static frc.robot.Constants.SwerveConstants.MAX_ROBOT_ROT_SPEED;
import static frc.robot.Constants.SwerveConstants.MAX_ROBOT_TRANS_SPEED;
import static frc.robot.Constants.SwerveConstants.PATHPLANNER_MAX_MOD_SPEED;
import static frc.robot.Constants.SwerveConstants.WHEEL_POSITIONS;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.HandledSleep;
public class Swerve extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(WHEEL_POSITIONS);
  SwerveDrivePoseEstimator multitagPoseEstimator;
  SwerveDriveOdometry wheelOdometry; 
  Vision vision;
  Field2d field = new Field2d();
  /* Want to put vision and path states on this field */
  Field2d vpField = new Field2d();
  boolean visionEnabled = true;

  int simGyro = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  SimDouble simGyroAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(simGyro, "Yaw"));
  double rotSpdSetpoint = 0;

  SwerveModule[] modules = {
    new SwerveModule(0, "FL"),
    new SwerveModule(1, "FR"),
    new SwerveModule(2, "BL"),
    new SwerveModule(3, "BR")
  };

  public Swerve() {
    this.vision = new Vision(this);
    gyro.zeroYaw();
    SmartDashboard.putData("Field", field);
    SmartDashboard.putData("Vision and Paths Field", vpField);

    /* Configure the motors in batch */
    for (SwerveModule module : modules) {
      module.factoryDefaults();
      HandledSleep.sleep(40);
    }

    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    for (SwerveModule module : modules) {
      module.configure();
      HandledSleep.sleep(100);
    }

    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);
    for (SwerveModule module : modules) {
      module.setupStatusFrames();
      HandledSleep.sleep(100);
    }
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    if(!DriverStation.isFMSAttached()) {
      SmartDashboard.putBoolean("Swerve/manualVoltageSteer", false);
      SmartDashboard.putBoolean("Swerve/manualVoltageDrive", false);
    }

    SparkMaxOdometryThread.getInstance().start();

    multitagPoseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));
    wheelOdometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), getModulePositions());
    resetPose(new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(180)));

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      Logger.recordOutput("Swerve/Pathplanner/CurrentPose", pose);
      vpField.getObject("Pathplanner Current Pose").setPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      Logger.recordOutput("Swerve/Pathplanner/TargetPose", pose);
      vpField.getObject("Pathplanner Target Pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      ArrayList<Trajectory.State> states = new ArrayList<>();
      
      /* I mean there's the edge case of there being exactly 1 state but idk why that would happen */
      if(poses.size() != 0) {
        Pose2d lastPose = poses.get(0);
        double t = 0;
        for(var pose: poses.subList(1, poses.size())) {
          Pose2d delta = new Pose2d(pose.getTranslation().minus(lastPose.getTranslation()), pose.getRotation().minus(lastPose.getRotation()));
          double curvature = delta.getRotation().getRadians() / delta.getTranslation().getNorm();
          states.add(new Trajectory.State(t, delta.getX(), delta.getY(), pose,curvature));
          t += 0.02; // not sure how the paths are spaced out so this might be wrong
        }
      } else {
        /* Just make it render probably offscreen (though just doing nothing and keeping the old one there is an option now that we have our own field for it) */
        states.add(new Trajectory.State(0, 0, 0, new Pose2d(-100, -100, new Rotation2d()), 0));
      }
      Trajectory traj = new Trajectory(states);
      Logger.recordOutput("Swerve/Pathplanner/ActivePath", traj);
      vpField.getObject("Pathplanner Path").setPoses(poses);
    });

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.7, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.7, 0.0, 0.0), // Rotation PID constants
                PATHPLANNER_MAX_MOD_SPEED, // Max module speed, in m/s
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

    SmartDashboard.putBoolean("Swerve/DesaturateWheelSpeeds", DESATURATE);
    SmartDashboard.putData("Swerve/PoseRotPID", headingController);
  }

  public Rotation2d getGyroRotation() {
    /* gyro's inverted i believe */
    return Rotation2d.fromRadians(gyro.getRotation2d().getRadians());
  }

  @Override
  public void simulationPeriodic() {
    // var degreesPerSecond = Units.radiansToDegrees(rotSpdSetpoint);
    var degreesPerSecond = Units.radiansToDegrees(getRobotRelativeSpeeds().omegaRadiansPerSecond);
    simGyroAngle.set(simGyroAngle.get() - (degreesPerSecond * 0.02));
  }

  public Command pathFindTo(Pose2d targetPose) {
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            Constants.AutoConstants.PATHFIND_CONSTRAINTS,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    return pathfindingCommand.withTimeout(5);
  }

  public Command goTo(Pose2d targetPose) {
    Rotation2d startHeading = targetPose.getTranslation().minus(getPose().getTranslation()).getAngle();
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(getPose().getTranslation(), startHeading),
        targetPose
    );

    PathPlannerPath ampPath = new PathPlannerPath(
        bezierPoints,
        AutoConstants.PATHFIND_CONSTRAINTS,
        new GoalEndState(0.0, targetPose.getRotation())
    );
    return AutoBuilder.followPath(ampPath);
  }

  PIDController headingController = new PIDController(5.2, 0, 0.5);

  public void drive(ChassisSpeeds speeds, Double alignToAngle) {

    // always calculating so that the pid controller can calculate the derivative
    double commandedRot = headingController.calculate(getPose().getRotation().getRadians());

    headingController.enableContinuousInput(0, 2 * Math.PI);
    headingController.setTolerance(0.005);

    if(alignToAngle != null) {
      Logger.recordOutput("Swerve/PoseRotPidAtSetpoint", headingController.atSetpoint());
      headingController.setSetpoint(alignToAngle);
      if(!headingController.atSetpoint()) {
        speeds.omegaRadiansPerSecond = commandedRot;
      } else {
        speeds.omegaRadiansPerSecond = 0;
      }
    }

    ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds, 0.02);
    rotSpdSetpoint = discretized.omegaRadiansPerSecond;
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discretized);
    if (SmartDashboard.getBoolean("Swerve/desaturateWheelSpeeds", DESATURATE)) {
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, discretized, MAX_MOD_SPEED, MAX_ROBOT_TRANS_SPEED, MAX_ROBOT_ROT_SPEED);
    }

    Logger.recordOutput("Swerve/Speeds", speeds);
    Logger.recordOutput("Swerve/DiscretizedSpeeds", discretized);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(moduleStates[i]);
    }
  }

  public void drive(ChassisSpeeds speeds) {
    drive(speeds, null);
  }

  public void driveFieldRelative(ChassisSpeeds speeds, Double alignToAngle) {
    var rot = getPose().getRotation();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      rot = rot.rotateBy(Rotation2d.fromDegrees(180));
    }
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rot), alignToAngle);
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    driveFieldRelative(speeds, null);
  }

  public SwerveModulePosition[] getModulePositions() {
    getModuleStates();
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getCurrentModulePosition();
    }
    Logger.recordOutput("Swerve/Positions", positions);
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getCurrentState();
    }
    Logger.recordOutput("Swerve/States", states);
    return states;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  @Override
  public void periodic() {

    for (SwerveModule module : modules) {
      module.periodic();
    }


    double[] timestamps;
    // indexed by module then time
    SwerveModulePosition[][] modPositions = new SwerveModulePosition[modules.length][];
    // indexed by time then module
    SwerveModulePosition[][] reshapedPositions;

    odometryLock.lock();
    try {
      timestamps = modules[0].getOdometryTimestamps();
      reshapedPositions = new SwerveModulePosition[timestamps.length][];
      
      for (int i = 0; i < modules.length; i++) {
        modPositions[i] = modules[i].getOdometryModPositions();
      }
    } finally {
      odometryLock.unlock();
    }

    // need to reshape the array such that you index it by time then module
    for (int time = 0; time < timestamps.length; time++) {
      reshapedPositions[time] = new SwerveModulePosition[modules.length];
      boolean skipThisBatch = false;
      for (int mod = 0; mod < modules.length; mod++) {
        if(modPositions[mod][time].distanceMeters == 0) {
          skipThisBatch = true;
          break;
        }
        reshapedPositions[time][mod] = modPositions[mod][time];
      }
      Logger.recordOutput("Swerve/SkippedBecauseZero", skipThisBatch);
      if(skipThisBatch) {
        continue;
      }
      multitagPoseEstimator.updateWithTime(timestamps[time], getGyroRotation(), reshapedPositions[time]);
      wheelOdometry.update(getGyroRotation(), reshapedPositions[time]);
    }

    if (getPose().getX() == Double.NaN) {
      DriverStation.reportWarning("We NaNed :(", false);
      resetPose(new Pose2d());
    }

    if(isVisionEnabled()) {
      boolean thereWereTags = vision.feedPoseEstimator(multitagPoseEstimator);
      if(DriverStation.isDisabled()) {
        if (thereWereTags) {
          LedState.setRainbowSpeed(6);
        } else {
          LedState.resetRainbowSpeed();
        }
      }
    }


    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/WheelOdom", wheelOdometry.getPoseMeters());
    /* Glass doesnt support struct fields really but they're nicer to use in AScope :( */
    Logger.recordOutput("Swerve/PoseX", getPose().getX());
    Logger.recordOutput("Swerve/PoseY", getPose().getY());
    Logger.recordOutput("Swerve/PoseRotation", getPose().getRotation().getRadians());
    Logger.recordOutput("Swerve/PoseRotationDegrees", getPose().getRotation().getDegrees());
    Logger.recordOutput("Swerve/CurrentSpeeds", getRobotRelativeSpeeds());
    Logger.recordOutput("Swerve/GyroAngle", -Units.degreesToRadians(gyro.getYaw()));
    Logger.recordOutput("Vision/VisionEnabled", isVisionEnabled());
    ShooterCalculations.getShooterAngle(getPose().getTranslation());
    ShooterCalculations.getYaw(getPose().getTranslation());
    field.setRobotPose(getPose());
    vpField.setRobotPose(getPose());

    if(!DriverStation.isFMSAttached()) {
      var steer = SmartDashboard.getBoolean("Swerve/manualVoltageSteer", false);
      var drive = SmartDashboard.getBoolean("Swerve/manualVoltageDrive", false);
      for (SwerveModule module : modules) {
        module.setSysidVoltageMode(drive, steer);
      }
    }
  }

  public Pose2d getPose() {
    return multitagPoseEstimator.getEstimatedPosition();
  }

  public Pose2d getPoseWheelsOnly() {
    return wheelOdometry.getPoseMeters();
  }

  public boolean atYaw(double yaw) {
    var difference = Math.abs(new Rotation2d(yaw).minus(getPose().getRotation()).getRadians());
    return difference < FACING_SPEAKER_TOLERANCE;
  }

  public Pose2d getPoseAllianceRelative() {
    Pose2d pose = getPose();
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    if(isRed) {
      return GeometryUtil.flipFieldPose(pose);
    }
    return pose;
  }
  
  public void resetPose(Pose2d pose) {
    for (SwerveModule module : modules) {
      module.resetEncoders();
    }
    multitagPoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    wheelOdometry.resetPosition(getGyroRotation(), getModulePositions(), pose);
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

  public boolean isVisionEnabled() {
    return visionEnabled;
  }

  public void disableVision() {
    this.visionEnabled = false;
  }

  public void enableVision() {
    this.visionEnabled = true;
  }

  public void setBrakeCoast(boolean willBrake) {
    for(var module: modules) {
      module.setBrakeCoast(willBrake);
    }
    Logger.recordOutput("Swerve/IsBraken", willBrake);
  }

  /* Not really volts */
  public SysIdRoutine getSysIdRoutineAngular() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(0.5).per(Seconds.of(1)), Volts.of(3.5), null,
        (state) -> Logger.recordOutput("SysIdTestState", state.toString())
      ),
      new Mechanism(
        (Measure<Voltage> v) -> {
          double pwr = v.in(Volts);
          drive(new ChassisSpeeds(0, 0, pwr));
        },
        null,
        this
      )
    );
  }
}
