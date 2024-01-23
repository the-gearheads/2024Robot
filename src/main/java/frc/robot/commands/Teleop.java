package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
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
    SmartDashboard.putBoolean("Teleop/HeadingPID", false);
    SmartDashboard.putData("Swerve/headingcontroller", headingController);

  }

  @Override
  public void execute() {
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    double rot = Controllers.driverController.getRotateAxis();

    var attemptingToRotate = MathUtil.applyDeadband(rot, 0.02) != 0; 

    double mod = Math.abs(Math.pow(Controllers.driverController.getSpeedModifierAxis(), 2));

    // x = Math.pow(x, 3);
    // y = Math.pow(y, 3);
    rot = Math.pow(rot, 3);

    x *= BASE_TRANS_SPEED;
    y *= BASE_TRANS_SPEED;
    rot *= BASE_ROT_SPEED;

    x += x * mod * MOD_TRANS_SPEED_FACTOR;
    y += y * mod * MOD_TRANS_SPEED_FACTOR;
    rot += rot * mod * MOD_ROT_SPEED_FACTOR;

    var speeds = new ChassisSpeeds(x, y, rot);
    if (SmartDashboard.getBoolean("Teleop/HeadingPID", false)) {
      headingPid(attemptingToRotate, speeds);
    }

    Logger.recordOutput("Swerve/Teleop/Speeds", speeds);
    swerve.driveFieldRelative(speeds);

    if(Controllers.driverController.getPatthfindButton().getAsBoolean()) {
      swerve.pathFindTo(swerve.getPose().plus(new Transform2d(new Translation2d(0.3, 0.3), swerve.getPose().getRotation()))).schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {

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