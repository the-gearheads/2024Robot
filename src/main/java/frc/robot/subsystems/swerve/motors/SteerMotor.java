package frc.robot.subsystems.swerve.motors;

import static frc.robot.Constants.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class SteerMotor {

  CANSparkMax max;
  SparkAbsoluteEncoder encoder;
  PIDController pid = new PIDController(STEER_PID[0], STEER_PID[1], STEER_PID[2]);

  int index;
  Rotation2d offset;
  String modulePath;

  boolean manualVoltageOnly = false;

  public SteerMotor(int id, int index, Rotation2d offset, String modulePath) {
    max = new CANSparkMax(id, MotorType.kBrushless);
    encoder = max.getAbsoluteEncoder(Type.kDutyCycle);
    this.index = index;
    this.offset = offset;
    this.modulePath = modulePath;

    pid.enableContinuousInput(0, 2 * Math.PI);
  }
  
  public void noOffsetSetAngle(Rotation2d angle) {
    pid.setSetpoint(angle.getRadians());
  }

  public void setAngle(Rotation2d angle) {
    noOffsetSetAngle(angle.plus(offset));
  }

  public Rotation2d noOffsetGetAngle() {
    return Rotation2d.fromRadians(encoder.getPosition());
  }

  public Rotation2d getAngle() {
    return noOffsetGetAngle().minus(offset);
  }

  public void periodic() {
    if (manualVoltageOnly) {
      return;
    }
    setVoltage(pid.calculate(encoder.getPosition()) + STEER_FEEDFORWARD.calculate(pid.getSetpoint()));
  }

  /* meant for sysid and stuff */
  public void setManualVoltageOnly(boolean manualVoltageOnly) {
    this.manualVoltageOnly = manualVoltageOnly;
  }

  public void setVoltage(double volts) {
    max.setVoltage(volts);
  }

  public void log() {
    Logger.recordOutput(modulePath + "/steerVolts", max.getAppliedOutput() * max.getBusVoltage());
    Logger.recordOutput(modulePath + "/steerAngle", getAngle().getRadians());
    Logger.recordOutput(modulePath + "/steerVel", encoder.getVelocity());
    Logger.recordOutput(modulePath + "/targetAngle", pid.getSetpoint());
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
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }
}
