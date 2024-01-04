package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  }

  @Override
  public void execute() {
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    double rot = Controllers.driverController.getRotateAxis();

    double mod = Math.pow(Controllers.driverController.getSpeedModifierAxis(), 3);

    x = Math.pow(x, 3);
    y = Math.pow(y, 3);
    rot = Math.pow(rot, 3);

    x *= BASE_TRANS_SPEED;
    y *= BASE_TRANS_SPEED;
    rot *= BASE_ROT_SPEED;

    x += mod * MOD_TRANS_SPEED_FACTOR;
    y += mod * MOD_TRANS_SPEED_FACTOR;
    rot += mod * MOD_ROT_SPEED_FACTOR;

    var speeds = new ChassisSpeeds(x, y, rot);
    Logger.recordOutput("/Swerve/Teleop/Speeds", speeds);
    swerve.driveFieldRelative(speeds);
  }

  @Override
  public void end(boolean interrupted) {

  }

}
