package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.DEFAULT_SPEED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class PrepareToShoot extends Command {
  Shooter shooter;
  Swerve swerve;
  Arm arm;

  public PrepareToShoot(Shooter shooter, Swerve swerve, Arm arm) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.arm = arm;
    addRequirements(shooter, arm);
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
    return shooter.atSpeed() && swerve.atSpeakerYaw() && MathUtil.isNear(arm.getAngle().getRadians(), shooterAngle, 0.01);
  }

}
