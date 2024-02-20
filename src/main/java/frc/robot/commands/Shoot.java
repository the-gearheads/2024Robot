package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.DEFAULT_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class Shoot extends Command {
  Shooter shooter;
  Feeder feeder;
  Swerve swerve;

  public Shoot(Shooter shooter, Feeder feeder, Swerve swerve) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.swerve = swerve;
    addRequirements(feeder, shooter);
  }

  @Override
  public void initialize() {
    shooter.setSpeed(DEFAULT_SPEED);
  }

  @Override
  public boolean isFinished() {
    return shooter.atSpeed() && swerve.atSpeakerYaw();
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      feeder.run();
      new WaitCommand(0.5);
    }
  }
}
