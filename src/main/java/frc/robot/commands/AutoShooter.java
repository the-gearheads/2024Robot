package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.Constants.ShooterConstants.*; //TODO: only import what you are actually using

public class AutoShooter extends Command {
  Shooter shooter;
  Swerve swerve;
  Feeder feeder;
  

  public AutoShooter(Shooter shooter, Swerve swerve, Feeder feeder) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.feeder = feeder;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d pose = swerve.getPose();
    double speakerDistance = ShooterCalculations.getDistanceToSpeaker(pose.getTranslation());
    if (speakerDistance < AUTO_SHOOTER_DISTANCE) {
      ShooterCalculations.setShooterPower(shooter);
    } else {
      shooter.setSpeed(0);
    }
  }
}
