package frc.robot.subsystems.swerve.motors;

import static frc.robot.Constants.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import java.util.OptionalDouble;

public class DriveMotor {

  CANSparkFlex flex;
  RelativeEncoder encoder;

  SparkPIDController pid;

  String modulePath;
  boolean manualVoltageOnly = false;
  double driveSetpoint = 0;

  int index;

  /* id is the CAN id, index is the index into the array of modules and stuff */
  public DriveMotor(int id, int index, String modulePath) {
    flex = new CANSparkFlex(id, MotorType.kBrushless);
    pid = flex.getPIDController();
    pid.setP((DRIVE_PID[0] / 12.0) * DRIVE_VEL_FACTOR);
    pid.setI(0);
    pid.setD(0);
    encoder = flex.getEncoder();
    this.modulePath = modulePath;
    this.index = index;
  }

  public void log() {
    Logger.recordOutput(modulePath + "/driveVolts", getVoltage());
    Logger.recordOutput(modulePath + "/drivePos", getPosition());
    Logger.recordOutput(modulePath + "/driveVel", getVelocity());
    Logger.recordOutput(modulePath + "/targetSpeed", driveSetpoint);
    Logger.recordOutput(modulePath + "/manualVoltageOnly", manualVoltageOnly);
  }

  public void setSpeed(double speed) {
    driveSetpoint = speed;
    pid.setReference(speed, ControlType.kVelocity, 0, DRIVE_FEEDFORWARD.calculate(speed));
  }

  public double getVoltage() {
    return flex.getAppliedOutput() * flex.getBusVoltage();
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

  public double getPosition() {
    return encoder.getPosition();
  }

  public OptionalDouble getPositionOptional() {
    double val = getPosition();
    if(flex.getLastError() == REVLibError.kOk) {
      return OptionalDouble.of(val);
    } else {
      return OptionalDouble.empty();
    }
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  /* call this first */
  public void factoryDefaults() {
    flex.restoreFactoryDefaults();
  }

  /* then this, after a delay */
  public void configure() {
    flex.setCANTimeout(250);
    flex.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
    flex.setIdleMode(IdleMode.kBrake);
    flex.setInverted(IS_INVERTED[index]);
    flex.enableVoltageCompensation(12);
    encoder.setPositionConversionFactor(DRIVE_POS_FACTOR);
    encoder.setVelocityConversionFactor(DRIVE_VEL_FACTOR);
  }

  /* then this, surrounded by 2 delays */
  public void setupStatusFrames() {
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int)(1000.0 / ODOMETRY_FREQUENCY));
    /* Don't have an analog encoder */
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* Don't have a duty cycle encoder */
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
    flex.setCANTimeout(0);
  }
}
