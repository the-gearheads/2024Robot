package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.Polygon;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class AutoShooter extends Command {
  Shooter shooter;
  Swerve swerve;
  Polygon shooterSpin = new Polygon(SHOOTER_SPIN_X, SHOOTER_SPIN_Y);

  public AutoShooter(Shooter shooter, Swerve swerve) {
    this.shooter = shooter;
    this.swerve = swerve;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d pose = swerve.getPose();
    if (shooterSpin.contains(pose)) {
      shooter.setSpeed(DEFAULT_SPEED);
    } else {
      shooter.setSpeed(0);
    }
  }
}
