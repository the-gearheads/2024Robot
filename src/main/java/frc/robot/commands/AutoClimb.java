package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;

public class AutoClimb extends Command {
  Climber climber;
  public AutoClimb(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    leftCurrentSpike = false;
    rightCurrentSpike = false;
  }

  boolean leftCurrentSpike, rightCurrentSpike;

  public final double MAX_CURRENT = 28;

  @Override
  public void execute() {
    double leftVal = climber.leftFilter.lastValue();
    double rightVal = climber.rightFilter.lastValue();
    
    leftCurrentSpike = leftVal > MAX_CURRENT ? true : leftCurrentSpike;
    rightCurrentSpike = rightVal > MAX_CURRENT ? true : rightCurrentSpike;


    if(leftCurrentSpike && rightCurrentSpike) {
      climber.down(); // both move down equally now
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
