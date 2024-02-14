package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.HandledSleep;

import static frc.robot.Constants.ArmConstants.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private CANSparkFlex mainFlex = new CANSparkFlex(MAIN_ARM_ID, MotorType.kBrushless);
  private CANSparkFlex followerFlex = new CANSparkFlex(FOLLOWER_ARM_ID, MotorType.kBrushless);
  PIDController pid = new PIDController(PID[0], PID[1], PID[2]);

  double simAngle = 0.79;
  Mechanism2d mech = new Mechanism2d(1, 1);
  // cad guesstimates cause ascope wants these in meters
  MechanismRoot2d root = mech.getRoot("Shooter", 0.1032, 0.1379);
  MechanismLigament2d armMech = root.append(new MechanismLigament2d("Arm", 0.6660, 45));
  MechanismLigament2d floorMech = root.append(new MechanismLigament2d("Floor", 0.7557, 0));

  SparkAbsoluteEncoder enc;
  public Arm() {
    mainFlex.restoreFactoryDefaults();
    followerFlex.restoreFactoryDefaults();
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);
    setupStatusFrames();
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    mainFlex.setInverted(true);
    followerFlex.setInverted(true);

    mainFlex.setSmartCurrentLimit(80);
    followerFlex.setSmartCurrentLimit(80);

    // mainFlex.enableVoltageCompensation(12);
    // followerFlex.enableVoltageCompensation(12);

    followerFlex.follow(mainFlex, true);

    mainFlex.setIdleMode(IdleMode.kBrake);
    followerFlex.setIdleMode(IdleMode.kBrake);

    this.enc = mainFlex.getAbsoluteEncoder(Type.kDutyCycle);
    enc.setPositionConversionFactor(ARM_POS_FACTOR);
    enc.setVelocityConversionFactor(ARM_POS_FACTOR * 60);

    if(!DriverStation.isFMSAttached()) {
      SmartDashboard.putBoolean("Arm/manualVoltageOnly", false);
    
    }
    // hi gavin and or michael if you're reading this i'm sorry for the mess i made in the arm subsystem i'm trying to fix it now i promise i'll do better next time i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry i'm sorry
    SmartDashboard.putNumber("Arm/manualVoltage", 0);

    armMech.setColor(new Color8Bit(255, 255, 0));
    floorMech.setColor(new Color8Bit(255, 128, 128));
  }

  @Override
  public void periodic() {

    log();
    double ff;
    // experimental https://gist.github.com/person4268/46710dca9a128a0eb5fbd93029627a6b
    if(Math.abs(Units.radiansToDegrees(getAngle().getRadians() - pid.getSetpoint())) > ARM_ANGLE_LIVE_FF_THRESHOLD) {
      ff = FEEDFORWARD.calculate(getAngle().getRadians(), 0);
    } else {
      ff = FEEDFORWARD.calculate(pid.getSetpoint(), 0);
    }
    double output = pid.calculate(enc.getPosition(), pid.getSetpoint()) + ff;

    Logger.recordOutput("Arm/attemptedOutput", output);

    // robot saving code
    if(output > 0 && getAngle().getDegrees() > MAX_ANGLE) {
      output = 0;
    }

    if(output < 0 && getAngle().getDegrees() < MIN_ANGLE) {
      output = 0;
    }

    double spDegrees = Units.radiansToDegrees(pid.getSetpoint());
    if(spDegrees < MIN_ANGLE || spDegrees > MAX_ANGLE) {
      output = 0;
    }

    if(!DriverStation.isFMSAttached() && SmartDashboard.getBoolean("Arm/manualVoltageOnly", false)) {
      // output = SmartDashboard.getNumber("Arm/manualVoltage", 0); // false for sysid reasons, idk how to better do this
    }

    mainFlex.setVoltage(output);
  }

  private void log() {
    Logger.recordOutput("Arm/Position", getAngle().getRadians());
    Logger.recordOutput("Arm/PositionDeg", getAngle().getDegrees());
    Logger.recordOutput("Arm/Velocity", getVelocity());
    Logger.recordOutput("Arm/Voltage", mainFlex.getAppliedOutput() * mainFlex.getBusVoltage());
    Logger.recordOutput("Arm/Setpoint", pid.getSetpoint());
    Logger.recordOutput("Arm/OutOfRange", getAngle().getDegrees() > MAX_ANGLE || getAngle().getDegrees() < MIN_ANGLE);
    double spDegrees = Units.radiansToDegrees(pid.getSetpoint());
    Logger.recordOutput("Arm/SetpointOutOfRange", spDegrees < MIN_ANGLE || spDegrees > MAX_ANGLE);
    armMech.setAngle(getAngle().getDegrees());
    Logger.recordOutput("Arm/Mechanism2d", mech);
  }

  public Rotation2d getAngle() {
    if(Robot.isSimulation()) return new Rotation2d(simAngle);
    return new Rotation2d(enc.getPosition());
  }

  public double getVelocity() {
    return enc.getVelocity();
  }

  public void setAngle(double angleRad) {
    angleRad = MathUtil.clamp(angleRad, Units.degreesToRadians(MIN_ANGLE), Units.degreesToRadians(MAX_ANGLE));
    if(Robot.isSimulation()) simAngle = angleRad;
    pid.setSetpoint(angleRad);
  }

  public void setVoltage(Measure<Voltage> volts) {
    mainFlex.setVoltage(volts.in(Volts));
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

  
  public void setupStatusFrames() {
    /* Status 0 governs applied output, faults, and whether is a follower. We don't care about that super much, so we increase it */
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    /* We don't care about our motor position, only what the encoder reads */
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
    /* Don't have an analog sensor */
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* We -really- care about our duty cycle encoder readings though. THE DEFAULT WAS 200MS */
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    mainFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }
}
