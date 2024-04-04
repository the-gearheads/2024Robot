package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
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
  Debouncer leftDebouncer = new Debouncer(0.4);
  Debouncer rightDebouncer = new Debouncer(0.4);

  public final double MAX_CURRENT = 20;

  @Override
  public void execute() {
    double leftCur = climber.getFilteredLeftCurrent();
    double rightCur = climber.getFilteredRightCurrent();
    
    leftCurrentSpike = leftDebouncer.calculate(leftCur > MAX_CURRENT) ? true : leftCurrentSpike;
    rightCurrentSpike = rightDebouncer.calculate(rightCur > MAX_CURRENT) ? true : rightCurrentSpike;

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
