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

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  public DriveMotor drive;
  public SteerMotor steer;
  Rotation2d offset;

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
  }

  public void setState(SwerveModuleState state) {
    Logger.recordOutput(modulePath + "/DesiredSwerveStatePreOpt", state);

    /* Could be wrong but i don't think we need any of the tomfoolery that was previously here */
    state = SwerveModuleState.optimize(state, steer.getAngle());

    Logger.recordOutput(modulePath + "/DesiredSwerveStatePostOpt", state);

    steer.setAngle(state.angle);
    drive.setSpeed(state.speedMetersPerSecond);
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(drive.getPosition(), steer.getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(drive.getVelocity(), steer.getAngle());
  }

  public void setDriveVolts(double volts) {
    steer.setAngle(new Rotation2d(0));
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
  
}
