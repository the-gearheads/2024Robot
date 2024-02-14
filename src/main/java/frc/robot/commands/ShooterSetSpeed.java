package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterSetSpeed extends Command {
  Shooter shooter;
  double speed;
  boolean topOnly;

  public ShooterSetSpeed(Shooter shooter, double speed) {
    this(shooter, speed, false);
  }

  public ShooterSetSpeed(Shooter shooter, double speed, boolean topOnly) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.speed = speed;
    this.topOnly = topOnly;
  }


  @Override
  public void initialize() {
    if (topOnly) {
      shooter.setTopSpeed(speed);
      shooter.setBottomSpeed(0);
    } else {
      shooter.setSpeed(speed);
    }
  }
  
  @Override
  public boolean isFinished() {
    return shooter.atSpeed();
  }
}
