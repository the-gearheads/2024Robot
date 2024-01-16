package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;
import frc.robot.util.HandledSleep;

import static frc.robot.Constants.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

// these configs are kinda long and we gotta do it twice so why not put it in its own file
public class ShooterMotor {

  public CANSparkFlex max;
  public SparkPIDController pid;
  public RelativeEncoder enc;

  public double targetSpeed;
  public double targetVolts;

  public ShooterMotor(int id) {
    max = new CANSparkFlex(id, CANSparkFlex.MotorType.kBrushless);
    max.restoreFactoryDefaults();
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    max.setSmartCurrentLimit(70);
    max.setInverted(true);
    max.setIdleMode(IdleMode.kCoast);

    pid = max.getPIDController();
    enc = max.getEncoder();

    pid.setP(PIDF[0]);
    pid.setI(PIDF[1]);
    pid.setD(PIDF[2]);
    pid.setFF(PIDF[3]);
  }

  public void setSpeed(double speed) {
    targetSpeed = speed;
    pid.setReference(speed, ControlType.kVelocity);
  }

  public double getVolts() {
    return max.getAppliedOutput() * max.getBusVoltage();
  }

  public double getVelocity() {
    return enc.getVelocity();
  }

  public double getVelocitySetpoint() {
    return targetSpeed;
  }

  public void setVolts(double volts) {
    targetVolts = volts; // tbh idk if they really differ
    pid.setReference(volts, ControlType.kVoltage);
  }

  public void log(String name) {
    Logger.recordOutput("Shooter/" + name + "/Velocty", getVelocity());
    Logger.recordOutput("Shooter/" + name + "/VelocitySetpoint", getVelocitySetpoint());
    Logger.recordOutput("Shooter/" + name + "/Volts", getVolts());
    Logger.recordOutput("Shooter/" + name + "/VoltsSetpoint", targetVolts);
  }
}
