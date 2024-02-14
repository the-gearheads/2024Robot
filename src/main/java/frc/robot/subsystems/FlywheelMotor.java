package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
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

  public FlywheelMotor(String name, int id, double[] PID, SimpleMotorFeedforward ff, boolean inverted) {
    pid = new PIDController(PID[0], PID[1], PID[2]);
    this.name = name;
    this.ff = ff;
    flex = new CANSparkFlex(id, CANSparkFlex.MotorType.kBrushless);
    flex.restoreFactoryDefaults();
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    flex.setSmartCurrentLimit(80);
    flex.setInverted(inverted);
    flex.setIdleMode(IdleMode.kCoast);

    // we're just not gonna set the position or velocity conversion factors because they default to rot(/min)

    enc = flex.getEncoder();
    enc.setAverageDepth(1);
    enc.setMeasurementPeriod(8);

    SmartDashboard.putBoolean(name + "/manualVoltageOnly", false);
  }

  public FlywheelMotor(String name, int id, double[] PID, SimpleMotorFeedforward ff) {
    this(name, id, PID, ff, true);
  }

  public void setSpeed(double speed) {
    pid.setSetpoint(speed);
  }

  public void periodic() {
    if(!DriverStation.isFMSAttached() && SmartDashboard.getBoolean(name + "/manualVoltageOnly", false)) {
      return;
    }
    if(pid.getSetpoint() == 0) {
      // Want to coast to a stop rather than brake
      flex.setVoltage(0);
      return;
    }
    flex.setVoltage(ff.calculate(pid.getSetpoint()) + pid.calculate(enc.getVelocity()));
  }

  public double getVolts() {
    return flex.getAppliedOutput() * flex.getBusVoltage();
  }

  public double getVelocity() {
    return enc.getVelocity();
  }

  public double getVelocitySetpoint() {
    return pid.getSetpoint();
  }

  public void setVolts(double volts) {
    targetVolts = volts; // tbh idk if they really differ
    flex.setVoltage(volts);
  }

  public void setVolts(Measure<Voltage> volts) {
    setVolts(volts.in(Volts));
  }

  public void log() {
    Logger.recordOutput(name + "/Velocty", getVelocity());
    Logger.recordOutput(name + "/VelocitySetpoint", getVelocitySetpoint());
    Logger.recordOutput(name + "/Volts", getVolts());
    Logger.recordOutput(name + "/VoltsSetpoint", targetVolts);
  }
}
