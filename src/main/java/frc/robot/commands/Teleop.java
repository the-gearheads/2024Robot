package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.swerve.Swerve;

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
    // todo: multiply by speeds, field relative setting, etc
    swerve.driveFieldRelative(new ChassisSpeeds(
      Controllers.driverController.getTranslateXAxis(),
      Controllers.driverController.getTranslateYAxis(),
      Controllers.driverController.getRotateAxis()
    ));
  }

  @Override
  public void end(boolean interrupted) {

  }

}
