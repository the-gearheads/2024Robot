package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;

public class AutoClimb extends Command {
  Climber climber;
  public AutoClimb(Climber climber) {
    addRequirements(climber);
  }

  @Override
  public void initialize() {

  }

  boolean leftCurrentSpike, rightCurrentSpike;

  LinearFilter leftFilter = LinearFilter.movingAverage(5);
  LinearFilter rightFilter = LinearFilter.movingAverage(5);

  public final double MAX_CURRENT = 22;

  @Override
  public void execute() {
    double leftVal = leftFilter.calculate(climber.getLeftCurrent());
    double rightVal = rightFilter.calculate(climber.getRightCurrent());

    Logger.recordOutput("AutoClimb/LeftCurrentFiltered", leftVal);
    Logger.recordOutput("AutoClimb/RightCurrentFiltered", rightVal);
    
    leftCurrentSpike = leftVal > MAX_CURRENT ? true : leftCurrentSpike;
    rightCurrentSpike = rightVal > MAX_CURRENT ? true : rightCurrentSpike;


    if(leftCurrentSpike && rightCurrentSpike) {
      climber.up();
    }

    if(!leftCurrentSpike) {
      climber.leftMotor.setSpeed(-ClimberConstants.SPEED);
    } else {
      climber.leftMotor.setSpeed(0);
    }

    if(!rightCurrentSpike) {
      climber.rightMotor.setSpeed(-ClimberConstants.SPEED);
    } else {
      climber.rightMotor.setSpeed(0);
    }

    Logger.recordOutput("AutoClimb/LeftCurrentSpike", leftCurrentSpike);
    Logger.recordOutput("AutoClimb/RightCurrentSpike", rightCurrentSpike);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climber.setSpeed(0);
  }
}
