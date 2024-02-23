package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.BetterBangBang;

import static frc.robot.Constants.Controllers.*;

import org.littletonrobotics.junction.Logger;

public class Teleop extends Command {

  Swerve swerve;
  
  public Teleop(Swerve swerve) {
    addRequirements(swerve);
    this.swerve = swerve;
  }

  @Override
  public void initialize() {
    headingController.setSetpoint(swerve.getGyroRotation().getRadians());
    SmartDashboard.putBoolean("Teleop/HeadingPID", true);
    SmartDashboard.putData("Swerve/headingcontroller", headingController);
    SmartDashboard.putBoolean("Swerve/FieldRelative", true);
  }

  @Override
  public void execute() {
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    double rot = Controllers.driverController.getRotateAxis();

    var attemptingToRotate = MathUtil.applyDeadband(rot, 0.02) != 0; 

    double speedMod = Math.abs(Math.pow(Controllers.driverController.getSpeedModifierAxis(), 2));

    x = Math.pow(x, 2) * Math.signum(x);
    y = Math.pow(y, 2) * Math.signum(y);
    rot = Math.pow(rot, 3);

    // maybe consider throwing a slewratelimiter here, will test when robot is present

    x *= BASE_TRANS_SPEED;
    y *= BASE_TRANS_SPEED;
    rot *= BASE_ROT_SPEED;

    x   += x   * speedMod * MOD_TRANS_SPEED_FACTOR;
    y   += y   * speedMod * MOD_TRANS_SPEED_FACTOR;
    rot += rot * speedMod * MOD_ROT_SPEED_FACTOR;

    var speeds = new ChassisSpeeds(x, y, rot);

    var forcedAngle = Controllers.driverController.getShootingPrepare().getAsBoolean() || Controllers.driverController.getShootButton().getAsBoolean() ?
                      ShooterCalculations.getYawToSpeaker(swerve.getPose().getTranslation()).getRadians() : null;
    if(forcedAngle != null) headingController.setSetpoint(swerve.getGyroRotation().getRadians());

    if (SmartDashboard.getBoolean("Teleop/HeadingPID", true)) {
      headingPid(attemptingToRotate, speeds);
    }

    Logger.recordOutput("Swerve/Teleop/Speeds", speeds);
    
    if (SmartDashboard.getBoolean("Swerve/FieldRelative", true)) {
      swerve.driveFieldRelative(speeds, forcedAngle);
    } else {
      swerve.drive(speeds, forcedAngle);
    }

  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0, 0, 0));
  }

  private double touchedRotateAt = Timer.getFPGATimestamp();
  private BetterBangBang headingController = new BetterBangBang();
  private void headingPid(boolean attemptingToRotate, ChassisSpeeds speeds) {
    headingController.setTolerance(0.06);
    touchedRotateAt = attemptingToRotate ? Timer.getFPGATimestamp() : touchedRotateAt;
    double timeSinceLastRotate = Timer.getFPGATimestamp() - touchedRotateAt;
    if (!attemptingToRotate && timeSinceLastRotate > 0.15) {
      double rot = headingController.calculate(swerve.getGyroRotation().getRadians());
      
      speeds.omegaRadiansPerSecond += rot;
    } else {
      headingController.setSetpoint(swerve.getGyroRotation().getRadians());
    }
  }
}