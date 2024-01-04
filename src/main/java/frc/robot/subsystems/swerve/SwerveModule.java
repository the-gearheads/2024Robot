package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  CANSparkMax driveMotor;
  SparkMaxPIDController drivePID;
  RelativeEncoder driveEncoder;
  CANSparkMax steerMotor;
  SparkMaxPIDController steerPID;
  SparkMaxAbsoluteEncoder steerEncoder;

  Rotation2d offset;
  /* Used for telemetry reasons */
  String modulePath;
  double targetSpeed;
  double targetAngle;

  public SwerveModule(int driveMotorId, int turnMotorId, double offsetDegrees, String moduleName) {

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    steerMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

    drivePID = driveMotor.getPIDController();
    steerPID = steerMotor.getPIDController();

    steerEncoder = steerMotor.getAbsoluteEncoder(Type.kDutyCycle);
    steerEncoder.setPositionConversionFactor(STEER_POS_FACTOR);
    steerEncoder.setVelocityConversionFactor(STEER_VEL_FACTOR);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(DRIVE_POS_FACTOR);
    driveEncoder.setVelocityConversionFactor(DRIVE_VEL_FACTOR);

    setupStatusFrames();

    // i think if we burnFlash we should throw in a Thread.sleep

    this.offset = Rotation2d.fromDegrees(offsetDegrees);
    this.modulePath = "Swerve/" + moduleName;
  }

  protected Rotation2d getAngle() {
    return Rotation2d.fromRadians(steerEncoder.getPosition()).minus(offset);
  }

  public void setAngle(Rotation2d angle) {
    targetAngle = angle.getRadians();
    steerPID.setReference(angle.getRadians(), ControlType.kPosition);
  }

  public void setSpeed(double speed) {
    targetSpeed = speed;
    drivePID.setReference(speed, ControlType.kVelocity);
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
    return driveEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getSteerVelocity() {
    return steerEncoder.getVelocity();
  }

  public SwerveModulePosition getState() {
    return new SwerveModulePosition(getPosition(), getAngle());
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  public void periodic() {
    Logger.recordOutput(modulePath + "/driveVolts", driveMotor.getAppliedOutput());
    Logger.recordOutput(modulePath + "/steerVolts", steerMotor.getAppliedOutput());

    Logger.recordOutput(modulePath + "/drivePos", getPosition());
    Logger.recordOutput(modulePath + "/driveVel", getDriveVelocity());

    Logger.recordOutput(modulePath + "/steerAngle", getAngle());
    Logger.recordOutput(modulePath + "/steerVel", getSteerVelocity());

    Logger.recordOutput(modulePath + "/targetSpeed", targetSpeed);
    Logger.recordOutput(modulePath + "/targetAngle", targetAngle);
  }
  

  private void setupStatusFrames() {
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    /* Don't have an analog encoder */
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* Don't have a duty cycle encoder */
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

    /* Status 0 governs applied output, faults, and whether is a follower. We don't care about that super much, so we increase it */
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    /* We don't care about our motor position, only what the encoder reads */
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    /* Don't have an analog sensor */
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* We -really- care about our duty cycle encoder readings though. THE DEFAULT WAS 200MS */
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }
}
