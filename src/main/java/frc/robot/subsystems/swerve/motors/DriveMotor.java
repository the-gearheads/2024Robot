package frc.robot.subsystems.swerve.motors;

import static frc.robot.Constants.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;

public class DriveMotor {

  CANSparkFlex flex;
  RelativeEncoder encoder;

  PIDController pid = new PIDController(DRIVE_PID[0], DRIVE_PID[1], DRIVE_PID[2]);

  String modulePath;
  boolean manualVoltageOnly = false;

  int index;

  /* id is the CAN id, index is the index into the array of modules and stuff */
  public DriveMotor(int id, int index, String modulePath) {
    flex = new CANSparkFlex(id, MotorType.kBrushless);
    encoder = flex.getEncoder();
    this.modulePath = modulePath;
    this.index = index;
  }

  public void log() {
    Logger.recordOutput(modulePath + "/driveVolts", flex.getAppliedOutput() * flex.getBusVoltage());
    Logger.recordOutput(modulePath + "/drivePos", getPosition());
    Logger.recordOutput(modulePath + "/driveVel", getVelocity());
    Logger.recordOutput(modulePath + "/targetSpeed", pid.getSetpoint());
    Logger.recordOutput(modulePath + "/manualVoltageOnly", manualVoltageOnly);
  }

  public void setSpeed(double speed) {
    pid.setSetpoint(speed);
  }

  public void periodic() {
    if (manualVoltageOnly) {
      return;
    }
    setVoltage(pid.calculate(getVelocity()) + DRIVE_FEEDFORWARD.calculate(pid.getSetpoint()));
  }

  /* meant for sysid and stuff */
  public void setManualVoltageOnly(boolean manualVoltageOnly) {
    this.manualVoltageOnly = manualVoltageOnly;
  }

  public void setVoltage(double volts) {
    flex.setVoltage(volts);
  }

  public double getPosition() {
    return encoder.getPosition();
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
    flex.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
    flex.setIdleMode(IdleMode.kBrake);
    flex.setInverted(IS_INVERTED[index]);
    encoder.setPositionConversionFactor(DRIVE_POS_FACTOR);
    encoder.setVelocityConversionFactor(DRIVE_VEL_FACTOR);
  }

  /* then this, surrounded by 2 delays */
  public void setupStatusFrames() {
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    /* Don't have an analog encoder */
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* Don't have a duty cycle encoder */
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
  }
}
