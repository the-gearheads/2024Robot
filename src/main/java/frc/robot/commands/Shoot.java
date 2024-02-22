package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.DEFAULT_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class Shoot extends Command {
  Shooter shooter;
  Feeder feeder;
  Swerve swerve;
  Arm arm;

  public Shoot(Shooter shooter, Feeder feeder, Swerve swerve, Arm arm) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.swerve = swerve;
    this.arm = arm;
    addRequirements(feeder, shooter, arm);
  }

  @Override
  public void initialize() {
    shooter.setSpeed(DEFAULT_SPEED);
  }

  @Override
  public void execute() {
    arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
  }

  @Override
  public boolean isFinished() {
    double shooterAngle = ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation());
    return shooter.atSpeed() && swerve.atSpeakerYaw() && arm.getAngle().getRadians() == shooterAngle;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      feeder.run();
      new WaitCommand(0.5);
    }
  }
}
