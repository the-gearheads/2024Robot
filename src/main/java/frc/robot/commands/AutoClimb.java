package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
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
    

    if (leftVal > 20) {
      
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }
}
