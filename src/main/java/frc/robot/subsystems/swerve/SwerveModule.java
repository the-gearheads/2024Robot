package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.HandledSleep;

import static frc.robot.Constants.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  CANSparkMax drive;
  SparkMaxPIDController drivePid;
  RelativeEncoder driveEnc;
  CANSparkMax steer;
  SparkMaxPIDController steerPid;
  SparkMaxAbsoluteEncoder steerEnc;

  Rotation2d offset;
  /* Used for telemetry reasons */
  String modulePath;
  double targetSpeed;
  double targetAngle;

  public SwerveModule(int id, String moduleName) {

    drive = new CANSparkMax(MOTOR_IDS[id][0], MotorType.kBrushless);
    steer = new CANSparkMax(MOTOR_IDS[id][1], MotorType.kBrushless);

    drive.restoreFactoryDefaults();
    steer.restoreFactoryDefaults();
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    drive.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
    steer.setSmartCurrentLimit(STEER_CURRENT_LIMIT);

    drive.setInverted(IS_INVERTED[id]);

    drive.setIdleMode(IdleMode.kBrake);
    steer.setIdleMode(IdleMode.kBrake);
    
    drivePid = drive.getPIDController();
    steerPid = steer.getPIDController();

    steerEnc = steer.getAbsoluteEncoder(Type.kDutyCycle);
    steerEnc.setPositionConversionFactor(STEER_POS_FACTOR);
    steerEnc.setVelocityConversionFactor(STEER_VEL_FACTOR);
    
    steerEnc.setInverted(true);

    driveEnc = drive.getEncoder();
    driveEnc.setPositionConversionFactor(DRIVE_POS_FACTOR);
    driveEnc.setVelocityConversionFactor(DRIVE_VEL_FACTOR);

    steerPid.setFeedbackDevice(steerEnc);
    steerPid.setPositionPIDWrappingEnabled(true);
    steerPid.setPositionPIDWrappingMinInput(-Math.PI);
    steerPid.setPositionPIDWrappingMaxInput(Math.PI);

    steerPid.setP(STEER_PIDF[0]);
    steerPid.setI(STEER_PIDF[1]);
    steerPid.setD(STEER_PIDF[2]);
    steerPid.setFF(STEER_PIDF[3]);

    drivePid.setP(DRIVE_PIDF[0]);
    drivePid.setI(DRIVE_PIDF[1]);
    drivePid.setD(DRIVE_PIDF[2]);
    drivePid.setFF(DRIVE_PIDF[3]);

    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);
    setupStatusFrames(); // pretty important that this doesnt get missed
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);


    // i think if we burnFlash we should throw in a Thread.sleep

    this.offset = Rotation2d.fromDegrees(WHEEL_OFFSETS[id]);
    this.modulePath = "Swerve/" + moduleName;
  }

  protected Rotation2d getAngle() {
    return Rotation2d.fromRadians(steerEnc.getPosition()).minus(offset);
  }

  public void setAngle(Rotation2d angle) {
    targetAngle = angle.getRadians();
    steerPid.setReference(angle.getRadians(), ControlType.kPosition);
  }

  public void setSpeed(double speed) {
    targetSpeed = speed;
    drivePid.setReference(speed, ControlType.kVelocity);
  }

  public void setState(SwerveModuleState state) {
    /* I don't think this is needed in 2024 wpilib but it can't hurt */
    state = new SwerveModuleState(state.speedMetersPerSecond, state.angle);

    state.angle = state.angle.plus(offset);
    /* taking advantage of optimize so the modules that are rotated 180deg dont have to have their
     * drive encoders inverted
     */
    state = SwerveModuleState.optimize(state, getAngle());

    setAngle(state.angle);
    setSpeed(state.speedMetersPerSecond);
  }

  public double getPosition() {
    return driveEnc.getPosition();
  }

  public double getDriveVelocity() {
    return driveEnc.getVelocity();
  }

  public double getSteerVelocity() {
    return steerEnc.getVelocity();
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getPosition(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getAngle());
  }

  /* A horrible consequence of the fact that we abuse optimize to determine whether a motor should be inverted */
  public void setVolts(double volts) {
    var state = new SwerveModuleState(1, new Rotation2d(0));
    state.angle = state.angle.plus(offset);
    state = SwerveModuleState.optimize(state, getAngle());
    setAngle(state.angle);
    if(state.speedMetersPerSecond == -1) volts *= -1;
    drivePid.setReference(volts, ControlType.kVoltage);
  }

  public void resetEncoders() {
    driveEnc.setPosition(0);
  }

  public void periodic() {
    Logger.recordOutput(modulePath + "/driveVolts", drive.getAppliedOutput());
    Logger.recordOutput(modulePath + "/steerVolts", steer.getAppliedOutput());

    Logger.recordOutput(modulePath + "/drivePos", getPosition());
    Logger.recordOutput(modulePath + "/driveVel", getDriveVelocity());

    Logger.recordOutput(modulePath + "/steerAngle", getAngle().getRadians());
    Logger.recordOutput(modulePath + "/steerVel", getSteerVelocity());

    Logger.recordOutput(modulePath + "/targetSpeed", targetSpeed);
    Logger.recordOutput(modulePath + "/targetAngle", targetAngle);
  }
  

  private void setupStatusFrames() {
    drive.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    /* Don't have an analog encoder */
    drive.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    drive.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* Don't have a duty cycle encoder */
    drive.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    drive.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

    /* Status 0 governs applied output, faults, and whether is a follower. We don't care about that super much, so we increase it */
    steer.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    /* We don't care about our motor position, only what the encoder reads */
    steer.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    /* Don't have an analog sensor */
    steer.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    steer.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* We -really- care about our duty cycle encoder readings though. THE DEFAULT WAS 200MS */
    steer.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    steer.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }
}
