package frc.robot.commands;


import static frc.robot.Constants.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class AutoShoot extends Command {
  Shooter shooter;
  Swerve swerve;
  Arm arm;
  Feeder feeder;

  public AutoShoot(Shooter shooter, Swerve swerve, Arm arm, Feeder feeder) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.arm = arm;
    this.feeder = feeder;
    addRequirements(shooter, arm);
  }

  @Override
  public void initialize() {
  }

  public boolean isAligned() {
    double targetYaw = ShooterCalculations.getYaw(swerve.getPose().getTranslation()).getRadians();
    double yawTolerance = ShooterCalculations.getYawTolerance(swerve.getPose().getTranslation());
    double shooterAngle = ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation(), true);
    double armTolerance = ShooterCalculations.getArmTolerance(swerve.getPose().getTranslation());
    ChassisSpeeds swerveSpeeds = swerve.getRobotRelativeSpeeds();
    boolean swerveStopped = swerveSpeeds.vxMetersPerSecond <= MAX_SHOOTING_SPEED_VX &&
                            swerveSpeeds.vyMetersPerSecond <= MAX_SHOOTING_SPEED_VY &&
                            swerveSpeeds.omegaRadiansPerSecond < MAX_SHOOTING_SPEED_ROT;

    boolean shooterAtSpeed = shooter.atSpeed();
    boolean atYaw = swerve.atYaw(targetYaw, yawTolerance);
    return shooterAtSpeed && atYaw && arm.atPoint(shooterAngle, armTolerance) && swerveStopped;
  }

  @Override
  public void execute() {
    ShooterCalculations.setShooterPower(shooter);
    arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation(), true));
    Logger.recordOutput("Arm/AngleTolerance", ShooterCalculations.getArmTolerance(swerve.getPose().getTranslation()));
    Logger.recordOutput("Arm/AtAngle", arm.atPoint(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()), ShooterCalculations.getArmTolerance(swerve.getPose().getTranslation())));
  }

  @Override
  public boolean isFinished() {
    return feeder.getBeamBreakSwitch().negate().getAsBoolean();
  }

}
