package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.swerve.Swerve;
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
    hPid.enableContinuousInput(-Math.PI, Math.PI); // verify whether this should be [-pi, pi] or [0, 2pi]
    hPid.setSetpoint(swerve.getGyroRotation().getRadians());
    SmartDashboard.putBoolean("Teleop/HeadingPID", false);
  }

  @Override
  public void execute() {
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    double rot = Controllers.driverController.getRotateAxis();

    double mod = Math.abs(Math.pow(Controllers.driverController.getSpeedModifierAxis(), 2));

    x = Math.pow(x, 3);
    y = Math.pow(y, 3);
    rot = Math.pow(rot, 3);

    x *= BASE_TRANS_SPEED;
    y *= BASE_TRANS_SPEED;
    rot *= BASE_ROT_SPEED;

    x += x * mod * MOD_TRANS_SPEED_FACTOR;
    y += y * mod * MOD_TRANS_SPEED_FACTOR;
    rot += rot * mod * MOD_ROT_SPEED_FACTOR;

    var speeds = new ChassisSpeeds(x, y, rot);
    if (SmartDashboard.getBoolean("Teleop/HeadingPID", false)) {
      headingPid(rot != 0, speeds);
    }

    Logger.recordOutput("Swerve/Teleop/Speeds", speeds);
    swerve.driveFieldRelative(speeds);
  }

  @Override
  public void end(boolean interrupted) {

  }

  private PIDController hPid = new PIDController(5, 0, 0);
  private double touchedRotateAt = Timer.getFPGATimestamp();
  private void headingPid(boolean attemptingToRotate, ChassisSpeeds speeds) {
    touchedRotateAt = attemptingToRotate ? Timer.getFPGATimestamp() : touchedRotateAt;
    double timeSinceLastRotate = Timer.getFPGATimestamp() - touchedRotateAt;
    if (attemptingToRotate && timeSinceLastRotate > 0.1) {
      hPid.setSetpoint(swerve.getGyroRotation().getRadians());
    } else {
        double rot = hPid.calculate(swerve.getGyroRotation().getRadians());
        rot = MathUtil.clamp(rot, -2.5, 2.5);
        rot = MathUtil.applyDeadband(rot, 0.1);
        speeds.omegaRadiansPerSecond += rot;
    }
  }

}
