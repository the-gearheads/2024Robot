package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.util.HandledSleep;

import static frc.robot.Constants.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

// these configs are kinda long and we gotta do it twice so why not put it in its own file
public class ShooterMotor {

  public CANSparkFlex flex;
  public RelativeEncoder enc;

  public double targetSpeed;
  public double targetVolts;

  PIDController pid = new PIDController(PID[0], PID[1], PID[2]);

  public ShooterMotor(int id) {
    flex = new CANSparkFlex(id, CANSparkFlex.MotorType.kBrushless);
    flex.restoreFactoryDefaults();
    HandledSleep.sleep(Constants.THREAD_SLEEP_TIME);

    flex.setSmartCurrentLimit(50);
    flex.setInverted(true);
    flex.setIdleMode(IdleMode.kCoast);

    enc = flex.getEncoder();
  }

  public void setSpeed(double speed) {
    targetSpeed = speed;
    if(speed == 0) {
      // Want to coast to a stop rather than brake
      flex.setVoltage(0);
      return;
    }
    flex.setVoltage(FEEDFORWARD.calculate(speed) + pid.calculate(enc.getVelocity(), speed));
  }

  public double getVolts() {
    return flex.getAppliedOutput() * flex.getBusVoltage();
  }

  public double getVelocity() {
    return enc.getVelocity();
  }

  public double getVelocitySetpoint() {
    return targetSpeed;
  }

  public void setVolts(double volts) {
    targetVolts = volts; // tbh idk if they really differ
    flex.setVoltage(volts);
  }

  public void log(String name) {
    Logger.recordOutput("Shooter/" + name + "/Velocty", getVelocity());
    Logger.recordOutput("Shooter/" + name + "/VelocitySetpoint", getVelocitySetpoint());
    Logger.recordOutput("Shooter/" + name + "/Volts", getVolts());
    Logger.recordOutput("Shooter/" + name + "/VoltsSetpoint", targetVolts);
  }
}
