package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.HandledSleep;
import frc.robot.util.ProfiledPIDControllerCustomPeriod;

import static frc.robot.Constants.ArmConstants.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private CANSparkFlex mainFlex = new CANSparkFlex(MAIN_ARM_ID, MotorType.kBrushless);
  private CANSparkFlex followerFlex = new CANSparkFlex(FOLLOWER_ARM_ID, MotorType.kBrushless);
  ProfiledPIDControllerCustomPeriod pid = new ProfiledPIDControllerCustomPeriod(PID[0], PID[1], PID[2], ARM_CONSTRAINTS, 0.02);

  SingleJointedArmSim armSim = new SingleJointedArmSim(LinearSystemId.identifyPositionSystem(SIM_FEEDFORWARD.kv, SIM_FEEDFORWARD.ka),
                                                       DCMotor.getNeoVortex(2), ARM_MOTOR_GEARING,
                                                       ARM_LENGTH, MIN_ANGLE-0.1, MAX_ANGLE+0.1, false, 0.79);

  DutyCycleEncoder enc = new DutyCycleEncoder(0);
  public Arm() {
    mainFlex.restoreFactoryDefaults();
    followerFlex.restoreFactoryDefaults();

    configure();

    // We're doing this ourselves (DutyCycleEncoder doesn't have an invert mode)
    enc.setPositionOffset(0);
    enc.setDistancePerRotation(1);

    if(!DriverStation.isFMSAttached()) {
      SmartDashboard.putBoolean("Arm/manualVoltageOnly", false);
      SmartDashboard.putBoolean("Arm/noVoltage", false);
    
    }
    // hi gavin and or michael if you're reading this i'm sorry for the mess i made in the arm subsystem i'm trying to fix it now i promise i'll do better next time i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry
    SmartDashboard.putNumber("Arm/manualVoltage", 0);

    // update arm sim once so it doesn't start at 0
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);
    if(Robot.isSimulation()) armSim.update(0.02);
    pid.reset(getAngle().getRadians());
    pid.setGoal(getAngle().getRadians());
  }

  public void configure() {
    mainFlex.setCANTimeout(250);
    followerFlex.setCANTimeout(250);
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);
    setupStatusFrames();
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    mainFlex.getForwardLimitSwitch(com.revrobotics.SparkLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(true);
    mainFlex.getReverseLimitSwitch(com.revrobotics.SparkLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(true);

    mainFlex.setSmartCurrentLimit(80);
    followerFlex.setSmartCurrentLimit(80);

    // mainFlex.enableVoltageCompensation(12);
    // followerFlex.enableVoltageCompensation(12);

    followerFlex.follow(mainFlex, true);

    mainFlex.setCANTimeout(0);
    followerFlex.setCANTimeout(0);
  }

  private double output = 0;
  private double lastTimestamp = Timer.getFPGATimestamp();
  public boolean runPid = true;

  @Override
  public void periodic() {
    log();
    double ff;
    // experimental https://gist.github.com/person4268/46710dca9a128a0eb5fbd93029627a6b not sure how needed this is for a trapezoidal profile
    if(Math.abs(Units.radiansToDegrees(getAngle().getRadians() - pid.getSetpoint().position)) > ARM_ANGLE_LIVE_FF_THRESHOLD) {
      ff = FEEDFORWARD.calculate(getAngle().getRadians(), pid.getSetpoint().velocity);
    } else {
      ff = FEEDFORWARD.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    }
    pid.setPeriod(Math.max(Timer.getFPGATimestamp() - lastTimestamp, 0.02)); // this probably works? i dont really have a way to test it
    output = pid.calculate(getAngle().getRadians()) + ff;

    Logger.recordOutput("Arm/attemptedOutput", output);

    //robot saving code
    if(output > 0 && getAngle().getRadians() > MAX_ANGLE) {
      output = 0;
    }

    if(output < 0 && getAngle().getRadians() < MIN_ANGLE) {
      output = 0;
    }

    if(pid.getSetpoint().position < MIN_ANGLE || pid.getSetpoint().position > MAX_ANGLE) {
      output = 0;
    }

    // Might as well just get as close as we can
    if(pid.getGoal().position < MIN_ANGLE || pid.getGoal().position > MAX_ANGLE) {
      pid.setGoal(MathUtil.clamp(pid.getGoal().position, MIN_ANGLE, MAX_ANGLE));
    }

    output = Math.abs(output) > 0.02 ? output : 0;

    if(!DriverStation.isFMSAttached() && SmartDashboard.getBoolean("Arm/manualVoltageOnly", false)) {
      output = SmartDashboard.getNumber("Arm/manualVoltage", 0); // false for sysid reasons, idk how to better do this
    }

    if (!runPid) {
      runPid = true;
      return; 
    };

    if(Robot.isSimulation()) {
      armSim.setInputVoltage(MathUtil.clamp(output, -12, 12));
      armSim.update(0.02);
    }

    if(!DriverStation.isFMSAttached() && SmartDashboard.getBoolean("Arm/noVoltage", false)) {
      return;
    }
    
    mainFlex.setVoltage(-output);
  }

  private void log() {
    Logger.recordOutput("Arm/Position", getAngle().getRadians());
    Logger.recordOutput("Arm/PositionDeg", getAngle().getDegrees());
    Logger.recordOutput("Arm/Velocity", getVelocity());
    Logger.recordOutput("Arm/Voltage", output);
    Logger.recordOutput("Arm/SetpointTrapezoidPosition", pid.getSetpoint().position);
    Logger.recordOutput("Arm/SetpointTrapezoidVelocity", pid.getSetpoint().velocity);
    Logger.recordOutput("Arm/GoalPosition", pid.getGoal().position);
    Logger.recordOutput("Arm/GoalVelocity", pid.getGoal().velocity);
    Logger.recordOutput("Arm/OutOfRange", getAngle().getRadians() > MAX_ANGLE || getAngle().getRadians() < MIN_ANGLE);
    Logger.recordOutput("Arm/SetpointOutOfRange", pid.getSetpoint().position < MIN_ANGLE || pid.getSetpoint().position > MAX_ANGLE);
    Logger.recordOutput("Arm/GoalOutOfRange", pid.getGoal().position < MIN_ANGLE || pid.getGoal().position > MAX_ANGLE);
    Logger.recordOutput("Arm/EncConnected", enc.isConnected());
    Logger.recordOutput("Arm/WhatTheOffsetShouldBeIfTheArmIsAt90Deg", getAngle().getRadians() + ARM_OFFSET - Units.degreesToRadians(90));
  }

  public Rotation2d getAngle() {
    if(Robot.isSimulation()) return new Rotation2d(armSim.getAngleRads());
    return new Rotation2d((1-enc.getAbsolutePosition())*ARM_POS_FACTOR - ARM_OFFSET);
  }

  public double getVelocity() {
    if(Robot.isSimulation()) return armSim.getVelocityRadPerSec();
    // return enc.getVelocity();
    return 0;
  }

  public void setAngle(double angleRad) {
    angleRad = MathUtil.clamp(angleRad, MIN_ANGLE, MAX_ANGLE);
    pid.setGoal(angleRad);
  }

  public void setVoltage(Measure<Voltage> volts) {
    if(Robot.isSimulation()) armSim.setInputVoltage(volts.in(Volts));
    mainFlex.setVoltage(-volts.in(Volts));
  }

  public void resetToCurrentPose() {
    pid.setGoal(getAngle().getRadians());
    pid.reset(getAngle().getRadians());
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.8).per(Seconds.of(1)), Volts.of(6), null, 
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
        this::setVoltage,
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
  }

  public void setBrakeCoast(boolean willBrake) {
    mainFlex.setCANTimeout(250);
    followerFlex.setCANTimeout(250);
    mainFlex.setIdleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    followerFlex.setIdleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    Logger.recordOutput("Arm/IsBraken", willBrake);
    mainFlex.setCANTimeout(0);
    followerFlex.setCANTimeout(0);
  }

  public boolean atPoint(double angle) {
    return MathUtil.isNear(getAngle().getRadians(), angle, 0.01);
  }

  public boolean atPoint(double angle, double tolerance) {
    return MathUtil.isNear(getAngle().getRadians(), angle, tolerance);
  }

  public void setupStatusFrames() {
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    /* We don't care about our motor position, only what the encoder reads */
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
    /* Don't have an analog sensor */
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
  }
}
