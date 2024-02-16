package frc.robot.subsystems.swerve.motors;

import static frc.robot.Constants.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;

public class SteerMotor {

  CANSparkMax max;
  SparkAbsoluteEncoder encoder;
  SparkPIDController pid;

  int index;
  Rotation2d offset;
  String modulePath;

  double targetAngle;

  boolean manualVoltageOnly = false;

  public SteerMotor(int id, int index, Rotation2d offset, String modulePath) {
    max = new CANSparkMax(id, MotorType.kBrushless);
    encoder = max.getAbsoluteEncoder(Type.kDutyCycle);
    pid = max.getPIDController();
    this.index = index;
    this.offset = offset;
    this.modulePath = modulePath;
  }
  
  public void noOffsetSetAngle(Rotation2d angle) {
    if(manualVoltageOnly) return;
    pid.setReference(angle.getRadians(), ControlType.kPosition);
  }

  public void setAngle(Rotation2d angle) {
    targetAngle = angle.getRadians();
    noOffsetSetAngle(angle.plus(offset));
  }

  public Rotation2d noOffsetGetAngle() {
    return Rotation2d.fromRadians(encoder.getPosition());
  }

  public Rotation2d getAngle() {
    return noOffsetGetAngle().minus(offset);
  }

  public double getAngleRadians() {
    return encoder.getPosition() - offset.getRadians();
  }

  public void periodic() {
  }

  /* meant for sysid and stuff */
  public void setManualVoltageOnly(boolean manualVoltageOnly) {
    this.manualVoltageOnly = manualVoltageOnly;
  }

  public void setVoltage(double volts) {
    pid.setReference(volts, ControlType.kVoltage);
  }

  public void log() {
    Logger.recordOutput(modulePath + "/steerVolts", max.getAppliedOutput() * max.getBusVoltage());
    Logger.recordOutput(modulePath + "/steerAngle", getAngle().getRadians());
    Logger.recordOutput(modulePath + "/steerVel", encoder.getVelocity());
    Logger.recordOutput(modulePath + "/targetAngle", targetAngle);
    Logger.recordOutput(modulePath + "/manualVoltageOnly", manualVoltageOnly);
  }

  public void factoryDefaults() {
    max.restoreFactoryDefaults();
  }

  public void configure() {
    max.setSmartCurrentLimit(STEER_CURRENT_LIMIT);

    max.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(STEER_POS_FACTOR);
    encoder.setVelocityConversionFactor(STEER_VEL_FACTOR);
    encoder.setInverted(true);

    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMinInput(0);
    pid.setPositionPIDWrappingMaxInput(Math.PI * 2);

    pid.setP(STEER_PIDF[0]);
    pid.setI(STEER_PIDF[1]);
    pid.setD(STEER_PIDF[2]);
    pid.setFF(STEER_PIDF[3]);

    pid.setFeedbackDevice(encoder);
  }

  public void setupStatusFrames() {
    /* Status 0 governs applied output, faults, and whether is a follower. We don't care about that super much, so we increase it */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    /* We don't care about our motor position, only what the encoder reads */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    /* Don't have an analog sensor */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* We -really- care about our duty cycle encoder readings though. THE DEFAULT WAS 200MS */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus5, (int)(1000.0 / ODOMETRY_FREQUENCY));
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }
}
