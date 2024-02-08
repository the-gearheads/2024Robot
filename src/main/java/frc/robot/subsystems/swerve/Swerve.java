package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.HandledSleep;

import static frc.robot.Constants.SwerveConstants.*;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

public class Swerve extends SubsystemBase {
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(WHEEL_POSITIONS);
  SwerveDrivePoseEstimator m_PoseEstimator;
  Vision vision;
  Field2d field = new Field2d();

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
    m_PoseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), getPose());

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      Logger.recordOutput("Swerve/Pathplanner/CurrentPose", pose);
      field.getObject("Pathplanner current pose").setPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      Logger.recordOutput("Swerve/Pathplanner/TargetPose", pose);
      field.getObject("Pathplanner target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      ArrayList<Trajectory.State> states = new ArrayList<>();
      double t = 0;
      Pose2d lastPose = poses.get(0);
      for(var pose: poses.subList(1, poses.size())) {
        Pose2d delta = new Pose2d(pose.getTranslation().minus(lastPose.getTranslation()), pose.getRotation().minus(lastPose.getRotation()));
        double curvature = delta.getRotation().getRadians() / delta.getTranslation().getNorm();
        states.add(new Trajectory.State(t, delta.getX(), delta.getY(), pose,curvature));
        t += 0.02;
      }
      Trajectory traj = new Trajectory(states);
      Logger.recordOutput("Swerve/Pathplanner/ActivePath", traj);
      field.getObject("Pathplanner path").setPoses(poses);
    });

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

  @Override
  public void simulationPeriodic() {
    var degreesPerSecond = Units.radiansToDegrees(rotSpdSetpoint);
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
  public void drive(ChassisSpeeds speeds) {
    ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds, 0.02);
    rotSpdSetpoint = discretized.omegaRadiansPerSecond;
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discretized);

    if (SmartDashboard.getBoolean("Swerve/desaturateWheelSpeeds", false)) {
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, discretized, MAX_MOD_SPEED, MAX_MOD_TRANS_SPEED, MAX_MOD_ROT_SPEED);
    }

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(moduleStates[i]);
    }
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    var rot = getPose().getRotation();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      rot = rot.rotateBy(Rotation2d.fromDegrees(180));
    }
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rot));
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
    m_PoseEstimator.update(getGyroRotation(), getModulePositions());
    Optional<EstimatedRobotPose> leftVision = vision.getGlobalPoseFromLeft();
    Optional<EstimatedRobotPose> rightVision = vision.getGlobalPoseFromLeft();
    if (leftVision.isPresent()) {
      m_PoseEstimator.addVisionMeasurement(leftVision.get().estimatedPose.toPose2d(), leftVision.get().timestampSeconds);
    }
    if (rightVision.isPresent()) {
      m_PoseEstimator.addVisionMeasurement(rightVision.get().estimatedPose.toPose2d(), rightVision.get().timestampSeconds);
    }

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
    return m_PoseEstimator.getEstimatedPosition();
  }
  
  public void resetPose(Pose2d pose) {
    for (SwerveModule module : modules) {
      module.resetEncoders();
    }
    m_PoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
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
