package frc.robot.commands;


import static frc.robot.Constants.ShooterConstants.*;//TODO: only import what you are actually using

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
    // The parameter length here is too long in the recordOutput - find better way of passing
    Logger.recordOutput("Arm/AngleTolerance", ShooterCalculations.getArmTolerance(swerve.getPose().getTranslation()));
    Logger.recordOutput("Arm/AtAngle", arm.atPoint(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()), ShooterCalculations.getArmTolerance(swerve.getPose().getTranslation())));

  }

  @Override
  public boolean isFinished() {
    // TODO: find a way to better set these doubles - maybe they need to go into their own model
    double targetYaw = ShooterCalculations.getYaw(swerve.getPose().getTranslation()).getRadians();
    double yawTolerance = ShooterCalculations.getYawTolerance(swerve.getPose().getTranslation());
    double shooterAngle = ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation(), true);
    double armTolerance = ShooterCalculations.getArmTolerance(swerve.getPose().getTranslation());
    ChassisSpeeds swerveSpeeds = swerve.getRobotRelativeSpeeds();
    boolean swerveStopped = swerveSpeeds.vxMetersPerSecond <= MAX_SHOOTING_SPEED_VX &&
                            swerveSpeeds.vyMetersPerSecond <= MAX_SHOOTING_SPEED_VY &&
                            swerveSpeeds.omegaRadiansPerSecond < MAX_SHOOTING_SPEED_ROT;

                            // TODO: break the return statement up into more readable lines
    return shooter.atSpeed() && swerve.atYaw(targetYaw, yawTolerance) && arm.atPoint(shooterAngle, armTolerance) && swerveStopped;
  }

}
