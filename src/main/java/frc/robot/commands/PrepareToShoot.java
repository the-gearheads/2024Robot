package frc.robot.commands;


import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class PrepareToShoot extends Command {
  Shooter shooter;
  Swerve swerve;
  Arm arm;
  boolean ends;

  public PrepareToShoot(Shooter shooter, Swerve swerve, Arm arm, boolean ends) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.arm = arm;
    this.ends = ends;
    addRequirements(shooter, arm);
  }
  public PrepareToShoot(Shooter shooter, Swerve swerve, Arm arm) {
    this(shooter, swerve, arm, true);
  }
  
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ShooterCalculations.setShooterPower(shooter);
    arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation(), true));
  }

  @Override
  public boolean isFinished() {
    double targetYaw = ShooterCalculations.getYaw(swerve.getPose().getTranslation()).getRadians();
    double shooterAngle = ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation(), true);
    ChassisSpeeds swerveSpeeds = swerve.getRobotRelativeSpeeds();
    boolean swerveStopped = swerveSpeeds.vxMetersPerSecond <= MAX_SHOOTING_SPEED_VX &&
                            swerveSpeeds.vyMetersPerSecond <= MAX_SHOOTING_SPEED_VY &&
                            swerveSpeeds.omegaRadiansPerSecond < MAX_SHOOTING_SPEED_ROT;

    return shooter.atSpeed() && swerve.atYaw(targetYaw) && arm.atPoint(shooterAngle) && swerveStopped;
  }

}
