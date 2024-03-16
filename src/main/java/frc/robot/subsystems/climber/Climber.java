package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.FlywheelMotor;

import static frc.robot.Constants.ClimberConstants.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;


public class Climber extends SubsystemBase {

  public FlywheelMotor leftMotor = new FlywheelMotor("Climber/Left", LEFT_ID, PID, FEEDFORWARD, true, true);
  public FlywheelMotor rightMotor = new FlywheelMotor("Climber/Right", RIGHT_ID, PID, FEEDFORWARD, false, true);

  public Climber() {
    this.setDefaultCommand(Commands.run(this::stop, this));
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    if (leftMotor.getPosition() >= MAX_DIST && leftMotor.getVelocitySetpoint() > 0
        || leftMotor.getPosition() <= MIN_DIST && leftMotor.getVelocitySetpoint() < 0){
      Logger.recordOutput("Climber/Left/OutOfRange", true);
      leftMotor.setSpeed(0);
    } else {
      Logger.recordOutput("Climber/Left/OutOfRange", false);
    }
    leftMotor.periodic();
    if (rightMotor.getPosition() >= MAX_DIST && rightMotor.getVelocitySetpoint() > 0
        || rightMotor.getPosition() <= MIN_DIST && rightMotor.getVelocitySetpoint() < 0){
      rightMotor.setSpeed(0);
      Logger.recordOutput("Climber/Right/OutOfRange", true);
    } else {
      Logger.recordOutput("Climber/Right/OutOfRange", false);
    } // life saving code, DO NOT DELETE <3
    rightMotor.periodic();
    leftMotor.log();
    rightMotor.log();
  }

  private void setSpeed(double speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
  }
  
  public void up() {
    setSpeed(SPEED);
  }

  public void stop() {
    setSpeed(0);
  }

  public void down() {
    setSpeed(-SPEED);
  }

  public void setBrakeCoast(boolean willBrake) {
    rightMotor.setBrakeCoast(willBrake);
    leftMotor.setBrakeCoast(willBrake);
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, 
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
        (voltage) -> { leftMotor.setVolts(voltage.in(Volts)); rightMotor.setVolts(voltage.in(Volts)); },
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
  }
}
