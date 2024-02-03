package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.HandledSleep;

import static frc.robot.Constants.ArmConstants.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private CANSparkFlex mainFlex = new CANSparkFlex(MAIN_ARM_ID, MotorType.kBrushless);
  private CANSparkFlex followerFlex = new CANSparkFlex(FOLLOWER_ARM_ID, MotorType.kBrushless);
  PIDController pid = new PIDController(FOLLOWER_ARM_ID, MAIN_ARM_ID, FOLLOWER_ARM_ID);
  ArmFeedforward feedforward = new ArmFeedforward(1, 2, 3, 4);

  SparkAbsoluteEncoder enc;
  public Arm() {
    mainFlex.restoreFactoryDefaults();
    followerFlex.restoreFactoryDefaults();
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    mainFlex.setSmartCurrentLimit(80);
    followerFlex.setSmartCurrentLimit(80);

    mainFlex.enableVoltageCompensation(12);
    followerFlex.enableVoltageCompensation(12);

    followerFlex.follow(mainFlex, true);

    mainFlex.setIdleMode(IdleMode.kBrake);
    followerFlex.setIdleMode(IdleMode.kBrake);

    this.enc = mainFlex.getAbsoluteEncoder(Type.kDutyCycle);
    enc.setPositionConversionFactor(ARM_POS_FACTOR);
    enc.setVelocityConversionFactor(ARM_POS_FACTOR * 60);

    if(!DriverStation.isFMSAttached()) {
      SmartDashboard.putBoolean("Arm/manualVoltageOnly", false);
    }

  }

  public void periodic() {
    double ff = feedforward.calculate(pid.getSetpoint(), 0);
    double output = pid.calculate(enc.getPosition(), pid.getSetpoint()) + ff;

    // robot saving code
    if(output > 0 && getAngle() > MAX_ANGLE) {
      output = 0;
    }

    if(output < 0 && getAngle() < MIN_ANGLE) {
      output = 0;
    }

    if(!DriverStation.isFMSAttached() && SmartDashboard.getBoolean("Arm/manualVoltageOnly", false)) {
      return; // bruh moment
    }

    mainFlex.setVoltage(output);
  }

  private void log() {
    Logger.recordOutput("Arm/Position", getAngle());
    Logger.recordOutput("Arm/Velocity", getVelocity());
    Logger.recordOutput("Arm/Voltage", mainFlex.getAppliedOutput() * mainFlex.getBusVoltage());
    Logger.recordOutput("Arm/Setpoint", pid.getSetpoint());
    Logger.recordOutput("Arm/OutOfRange", getAngle() > MAX_ANGLE || getAngle() < MIN_ANGLE);
  }

  public double getAngle() {
    return enc.getPosition();
  }

  public double getVelocity() {
    return enc.getVelocity();
  }

  public void setAngle(double angleRad) {
    MathUtil.clamp(angleRad, MIN_ANGLE, MAX_ANGLE);
    pid.setSetpoint(angleRad);
  }

  public void setVoltage(Measure<Voltage> volts) {
    mainFlex.setVoltage(volts.in(Volts));
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, 
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
        this::setVoltage,
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
  }

  
}
