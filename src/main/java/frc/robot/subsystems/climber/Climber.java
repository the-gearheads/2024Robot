package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.FlywheelMotor;

import static frc.robot.Constants.ClimberConstants.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;


public class Climber extends SubsystemBase {

  public FlywheelMotor leftMotor = new FlywheelMotor("Climber/Left", LEFT_ID, PID, FEEDFORWARD, true, true, GEAR_RATIO);
  public FlywheelMotor rightMotor = new FlywheelMotor("Climber/Right", RIGHT_ID, PID, FEEDFORWARD, false, true, GEAR_RATIO);

  LinearFilter leftFilter = LinearFilter.movingAverage(10);
  LinearFilter rightFilter = LinearFilter.movingAverage(10);

  double lastFilteredLeftCurrent = 0;
  double lastFilteredRightCurrent = 0;

  public Climber() {
    this.setDefaultCommand(Commands.run(this::stop, this));
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
    leftFilter.calculate(0); // lastValue no work if its never been called before.
    rightFilter.calculate(0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/ControllerVal", Controllers.operatorController.getClimberProportion());
    if (leftMotor.getPosition() >= MAX_DIST && leftMotor.getVelocitySetpoint() > 0
        || leftMotor.getPosition() <= MIN_DIST && leftMotor.getVelocitySetpoint() < 0){
      Logger.recordOutput("Climber/Left/OutOfRange", true);
      if(!Controllers.driverController.allowClimberOverride().getAsBoolean()) {
        leftMotor.setSpeed(0);
      }
    } else {
      Logger.recordOutput("Climber/Left/OutOfRange", false);
    }
    leftMotor.periodic();
    if (rightMotor.getPosition() >= MAX_DIST && rightMotor.getVelocitySetpoint() > 0
        || rightMotor.getPosition() <= MIN_DIST && rightMotor.getVelocitySetpoint() < 0){
      if(!Controllers.driverController.allowClimberOverride().getAsBoolean()) {
        rightMotor.setSpeed(0);
      }
      Logger.recordOutput("Climber/Right/OutOfRange", true);
    } else {
      Logger.recordOutput("Climber/Right/OutOfRange", false);
    } // life saving code, DO NOT DELETE <3
    rightMotor.periodic();
    leftMotor.log();
    rightMotor.log();


    lastFilteredLeftCurrent = leftFilter.calculate(getLeftCurrent());
    lastFilteredRightCurrent = rightFilter.calculate(getRightCurrent());
    Logger.recordOutput("Climber/Left/FilteredCurrent", lastFilteredLeftCurrent);
    Logger.recordOutput("Climber/Right/FilteredCurrent", lastFilteredRightCurrent); 
  }

  public void setSpeed(double speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
  }

  public void up(double proportion) {
    setSpeedProportional(proportion, SPEED);
  }
  
  public void up() {
    up(0);
  }

  public void stop() {
    setSpeed(0);
  }

  public double getLeftCurrent() {
    return leftMotor.getMotorCurrent();
  }

  public double getRightCurrent() {
    return rightMotor.getMotorCurrent();
  }

  public double getFilteredLeftCurrent() {
    return lastFilteredLeftCurrent;
  }

  public double getFilteredRightCurrent() {
    return lastFilteredRightCurrent;
  }

  /**
   * @param proportion -1 to 1 (-1 only left moves, 1 only right moves)
   */
  public void setSpeedProportional(double proportion, double speed) {
    proportion = MathUtil.clamp(proportion, -1, 1);

    double leftMotorSpeed = speed * (1 + proportion); // This will range from 0 (at proportion = 1) to -2*SPEED (at proportion = -1)
    double rightMotorSpeed = speed * (1 - proportion); // This will range from -2*SPEED (at proportion = 1) to 0 (at proportion = -1)

    // Clamp speeds to not exceed SPEED in magnitude (might wanna fine tune this? could replace -SPEED with -SPEED*1.2)
    double absSpeed = Math.abs(speed);
    leftMotorSpeed = MathUtil.clamp(leftMotorSpeed, -absSpeed, absSpeed);
    rightMotorSpeed = MathUtil.clamp(rightMotorSpeed, -absSpeed, absSpeed);


    leftMotor.setSpeed(leftMotorSpeed);
    rightMotor.setSpeed(rightMotorSpeed);

    Logger.recordOutput("Climber/Proportion", proportion);
  }

  public void down(double proportion) {
    setSpeedProportional(proportion, -SPEED);
  }

  public void down() {
    down(0);
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
