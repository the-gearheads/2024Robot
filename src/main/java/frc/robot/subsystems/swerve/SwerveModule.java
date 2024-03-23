package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.motors.DriveMotor;
import frc.robot.subsystems.swerve.motors.DriveMotorSim;
import frc.robot.subsystems.swerve.motors.SteerMotor;
import frc.robot.subsystems.swerve.motors.SteerMotorSim;

import static frc.robot.Constants.SwerveConstants.*;

import java.util.Queue;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  public DriveMotor drive;
  public SteerMotor steer;
  Rotation2d offset;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  String modulePath;

  public SwerveModule(int id, String moduleName) {
    this.modulePath = "Swerve/" + moduleName;
    this.offset = Rotation2d.fromDegrees(WHEEL_OFFSETS[id]);
    if (Robot.isReal()) {
      drive = new DriveMotor(MOTOR_IDS[id][0], id, modulePath);
      steer = new SteerMotor(MOTOR_IDS[id][1], id, offset, modulePath);
    } else {
      drive = new DriveMotorSim(MOTOR_IDS[id][0], id, modulePath);
      steer = new SteerMotorSim(MOTOR_IDS[id][1], id, offset, modulePath);
    }
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(drive::getPositionOptional);
    turnPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(steer::getAngleRadiansOptional);
  }

  public void setState(SwerveModuleState state) {
    Logger.recordOutput(modulePath + "/DesiredSwerveStatePreOpt", state);

    /* Could be wrong but i don't think we need any of the tomfoolery that was previously here */
    state = SwerveModuleState.optimize(state, steer.getAngle());

    Logger.recordOutput(modulePath + "/DesiredSwerveStatePostOpt", state);

    steer.setAngle(state.angle);
    drive.setSpeed(state.speedMetersPerSecond);
  }

  /* setState except the speed is interpretedd as voltage */
  public void setStateVoltage(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, steer.getAngle());

    steer.setAngle(state.angle);
    drive.setVoltage(state.speedMetersPerSecond);
  }

  public SwerveModulePosition getCurrentModulePosition() {
    return new SwerveModulePosition(drive.getPosition(), steer.getAngle());
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(drive.getVelocity(), steer.getAngle());
  }

  public void setDriveVolts(double volts) {
    // steer.setAngle(new Rotation2d(0));
    drive.setVoltage(volts);
  }

  public void setSteerVolts(double volts) {
    steer.setVoltage(volts);
    drive.setVoltage(0);
  }

  /* Makes it such that PID doesn't run and you can set the voltage in peace */
  public void setSysidVoltageMode(boolean drive, boolean steer) {
    this.drive.setManualVoltageOnly(drive);
    this.steer.setManualVoltageOnly(steer);
  }

  public void resetEncoders() {
    drive.resetEncoder();
  }

  public void periodic() {
    steer.periodic();
    drive.periodic();
    steer.log();
    drive.log();
  }

  /* MUST CALL THESE THREE BEFORE USING */
  public void factoryDefaults() {
    steer.factoryDefaults();
    drive.factoryDefaults();
  }

  public void configure() {
    steer.configure();
    drive.configure();
  }

  public void setupStatusFrames() {
    steer.setupStatusFrames();
    drive.setupStatusFrames();
  }

  private Rotation2d[] getOdometrySteerPositions() {
    var angles =  turnPositionQueue
      .stream()
      .map((Double value) -> Rotation2d.fromRadians(value))
      .toArray(Rotation2d[]::new);
    Logger.recordOutput(modulePath + "/odometrySteerPositions", angles);
    turnPositionQueue.clear();
    return angles;
  }

  private double[] getOdometryDrivePositions() {
    var positions =  drivePositionQueue
      .stream()
      .mapToDouble((Double value) -> value)
      .toArray();
    Logger.recordOutput(modulePath + "/odometryDrivePositions", positions);
    drivePositionQueue.clear();
    return positions;
  }

  public double[] getOdometryTimestamps() {
    var timestamps =  timestampQueue
      .stream()
      .mapToDouble((Double value) -> value)
      .toArray();
    Logger.recordOutput(modulePath + "/odometryTimestamps", timestamps);
    timestampQueue.clear();
    return timestamps;
  }

  public SwerveModulePosition[] getOdometryModPositions() {
    var drivePositions = getOdometryDrivePositions();
    var turnPositions = getOdometrySteerPositions();
    var positions = new SwerveModulePosition[drivePositions.length];
    for (int i = 0; i < drivePositions.length; i++) {
      positions[i] = new SwerveModulePosition(drivePositions[i], turnPositions[i]);
    }
    return positions;
  }

  public void setBrakeCoast(boolean willBrake) {
    drive.setBrakeCoast(willBrake);
    steer.setBrakeCoast(willBrake);
  }  
}
