package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.DEFAULT_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
  Shooter shooter;
  Feeder feeder;

  public Shoot(Shooter shooter, Feeder feeder) {
    this.shooter = shooter;
    this.feeder = feeder;
    addRequirements(feeder, shooter);
  }

  @Override
  public void initialize() {
    shooter.setSpeed(DEFAULT_SPEED);
  }

  @Override
  public boolean isFinished() {
    return shooter.atSpeed();
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      feeder.run();
      new WaitCommand(0.5);
    }
  }
}
