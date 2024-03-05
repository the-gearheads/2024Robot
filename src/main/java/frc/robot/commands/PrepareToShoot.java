package frc.robot.commands;


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
  }

  @Override
  public void execute() {
    ShooterCalculations.setShooterPower(shooter);
    arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
  }

  @Override
  public boolean isFinished() {
    double targetYaw = ShooterCalculations.getYaw(swerve.getPose().getTranslation()).getRadians();
    double shooterAngle = ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation());
    return shooter.atSpeed() && swerve.atYaw(targetYaw) && arm.atPoint(shooterAngle);
  }

}
