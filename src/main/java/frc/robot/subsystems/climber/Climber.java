package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.FlywheelMotor;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ShooterConstants.DEFAULT_SPEED;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;


public class Climber extends SubsystemBase {

  public FlywheelMotor leftMotor = new FlywheelMotor("Climber/Left", LEFT_ID, PID, FEEDFORWARD, false, false);
  public FlywheelMotor rightMotor = new FlywheelMotor("Climber/Right", RIGHT_ID, PID, FEEDFORWARD, false, false);

  public Climber() {
    this.setDefaultCommand(Commands.run(this::stop, this));
  }

  @Override
  public void periodic() {
    if (leftMotor.getPosition() >= MAX_DIST && leftMotor.getVelocitySetpoint() > 0
        || leftMotor.getPosition() <= MIN_DIST && leftMotor.getVelocitySetpoint() < 0){
      leftMotor.setSpeed(0);
    }
    leftMotor.periodic();
    if (rightMotor.getPosition() >= MAX_DIST && rightMotor.getVelocitySetpoint() > 0
        || rightMotor.getPosition() <= MIN_DIST && rightMotor.getVelocitySetpoint() < 0){
      rightMotor.setSpeed(0);
    }  // life saving code, DO NOT DELETE <3
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
