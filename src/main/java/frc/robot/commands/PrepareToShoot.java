package frc.robot.commands;


import static frc.robot.Constants.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

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
  double endAtSpeed;

  public PrepareToShoot(Shooter shooter, Swerve swerve, Arm arm, double endAtSpeed) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.arm = arm;
    this.endAtSpeed = endAtSpeed;
    addRequirements(shooter, arm);
  }
  public PrepareToShoot(Shooter shooter, Swerve swerve, Arm arm) {
    this(shooter, swerve, arm, -1);
  }
  
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ShooterCalculations.setShooterPower(shooter);
    arm.setAngle(ShooterCalculations.getShooterAngle(swerve, true));
    Logger.recordOutput("Arm/AngleTolerance", ShooterCalculations.getArmTolerance(swerve.getPose().getTranslation()));
    Logger.recordOutput("Arm/AtAngle", arm.atPoint(ShooterCalculations.getShooterAngle(swerve), ShooterCalculations.getArmTolerance(swerve.getPose().getTranslation())));

  }

  @Override
  public boolean isFinished() {
    double targetYaw = ShooterCalculations.getYaw(swerve).getRadians();
    double yawTolerance = ShooterCalculations.getYawTolerance(swerve.getPose().getTranslation());
    double shooterAngle = ShooterCalculations.getShooterAngle(swerve, true);
    double armTolerance = ShooterCalculations.getArmTolerance(swerve.getPose().getTranslation());
    ChassisSpeeds swerveSpeeds = swerve.getRobotRelativeSpeeds();
    boolean swerveStopped = swerveSpeeds.vxMetersPerSecond <= MAX_SHOOTING_SPEED_VX &&
                            swerveSpeeds.vyMetersPerSecond <= MAX_SHOOTING_SPEED_VY &&
                            swerveSpeeds.omegaRadiansPerSecond < MAX_SHOOTING_SPEED_ROT;

    boolean shooterAtSpeed = endAtSpeed == -1 ? shooter.atSpeed() : shooter.atSpeed(endAtSpeed, endAtSpeed);
    return shooterAtSpeed && swerve.atYaw(targetYaw, yawTolerance) && arm.atPoint(shooterAngle, armTolerance) && swerveStopped;
  }

}
