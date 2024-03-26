package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.HandledSleep;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

// these configs are kinda long and we gotta do it twice so why not put it in its own file
public class FlywheelMotor {

  public CANSparkFlex flex;
  public RelativeEncoder enc;

  public double targetVolts;

  PIDController pid;
  SimpleMotorFeedforward ff;

  String name;

  FlywheelSim sim;
  double simShooterPos = 0;

  boolean inverted, brakeMode;

  public FlywheelMotor(String name, int id, double[] PID, SimpleMotorFeedforward ff, boolean inverted, boolean brakeMode, double simGearRatio) {
    pid = new PIDController(PID[0], PID[1], PID[2]);
    this.name = name;
    this.ff = ff;
    this.inverted = inverted;
    this.brakeMode = brakeMode;
    flex = new CANSparkFlex(id, CANSparkFlex.MotorType.kBrushless);
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);
    flex.restoreFactoryDefaults();
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    flex.setSmartCurrentLimit(80);
    flex.setInverted(inverted);
    if(brakeMode) {
      flex.setIdleMode(IdleMode.kBrake);
    } else {
      flex.setIdleMode(IdleMode.kCoast);
    }

    // we're just not gonna set the position or velocity conversion factors because they default to rot(/min)

    enc = flex.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
    // enc = flex.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    configure();

    if(Robot.isSimulation()) {
      sim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(ff.kv, ff.ka), DCMotor.getNeoVortex(1), simGearRatio);
    }

    SmartDashboard.putBoolean(name + "/manualVoltageOnly", false);
  }

  public FlywheelMotor(String name, int id, double[] PID, SimpleMotorFeedforward ff) {
    this(name, id, PID, ff, true, false);
  }

  public FlywheelMotor(String name, int id, double[] PID, SimpleMotorFeedforward ff, boolean inverted, boolean brakeMode) {
    this(name, id, PID, ff, inverted, brakeMode, 1);
  }

  public void configure() {
    // we're just not gonna set the position or velocity conversion factors because they default to rot(/min)
    flex.setCANTimeout(250);
    flex.setSmartCurrentLimit(65);
    flex.setInverted(inverted);
    HandledSleep.sleep(100);
    if(brakeMode) {
      flex.setIdleMode(IdleMode.kBrake);
    } else {
      flex.setIdleMode(IdleMode.kCoast);
    }
    HandledSleep.sleep(100);
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    flex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    enc.setAverageDepth(1);
    enc.setMeasurementPeriod(8);
    flex.setCANTimeout(0);
  }

  public void setSpeed(double speed) {
    pid.setSetpoint(speed);
  }

  double superSpeedCount = 0;
  double lastReconfigured = Timer.getFPGATimestamp();

  public void periodic() {
    if(Robot.isSimulation()) {
      var maxVolts = RobotController.getBatteryVoltage();
      sim.setInputVoltage(MathUtil.clamp(targetVolts, -maxVolts, maxVolts));
      if(DriverStation.isDisabled()) {
        sim.setInputVoltage(0);
      }
      sim.update(0.02);
      simShooterPos += sim.getAngularVelocityRadPerSec() * 0.02;
    }

    if(!DriverStation.isFMSAttached() && SmartDashboard.getBoolean(name + "/manualVoltageOnly", false)) {
      return;
    }

    if(pid.getSetpoint() == 0) {
      // Want to coast to a stop rather than brake
      setVolts(0);
      return;
    }
    setVolts(ff.calculate(pid.getSetpoint()) + pid.calculate(enc.getVelocity()));

    if(Math.abs(enc.getVelocity()) > 8000) {
      superSpeedCount++;
    } else {
      superSpeedCount = 0;
    }

    if(superSpeedCount > 10) {
      DriverStation.reportWarning("FlywheelMotor " + name + " is like way too fast, reconfiguring", false);
      if(Timer.getFPGATimestamp() - lastReconfigured > 1) {
        superSpeedCount = 0;
        lastReconfigured = Timer.getFPGATimestamp();
        configure();
      } else {
        DriverStation.reportWarning("FlywheelMotor " + name + " reconfiguring, but we just did that, so not trying again", false);
      }
    }
  }

  public double getVolts() {
    if(Robot.isSimulation()) return targetVolts;
    return flex.getAppliedOutput() * flex.getBusVoltage();
  }

  public double getVelocity() {
    if(Robot.isSimulation()) return sim.getAngularVelocityRadPerSec(); // constants actually output in rpm so this function name is wrong btw
    return enc.getVelocity();
  }

  public double getVelocitySetpoint() {
    return pid.getSetpoint();
  }

  public void setVolts(double volts) {
    targetVolts = volts;
    flex.setVoltage(volts);
  }

  public void setPosition(double position) {
    enc.setPosition(position);
  }

  public void setBrakeCoast(boolean willBrake) {
    flex.setCANTimeout(250);
    flex.setIdleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    Logger.recordOutput(name + "/IsBraken", willBrake);
    flex.setCANTimeout(0);
  }

  public double getPosition() {
    if(Robot.isSimulation()) return simShooterPos;
    return enc.getPosition();
  }

  public void setVolts(Measure<Voltage> volts) {
    setVolts(volts.in(Volts));
  }

  public void log() {
    Logger.recordOutput(name + "/Velocty", getVelocity());
    Logger.recordOutput(name + "/VelocitySetpoint", getVelocitySetpoint());
    Logger.recordOutput(name + "/Position", getPosition());
    Logger.recordOutput(name + "/Volts", getVolts());
    Logger.recordOutput(name + "/VoltsSetpoint", targetVolts);
    Logger.recordOutput(name + "/LastReconfigured", lastReconfigured);
    Logger.recordOutput(name + "/SuperSpeedCount", superSpeedCount);
  }
}
