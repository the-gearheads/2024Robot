package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveAlignToSpeaker extends Command {
  Swerve swerve;

  public SwerveAlignToSpeaker(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double speakerAngle = ShooterCalculations.getYawToSpeaker(swerve.getPose().getTranslation()).getRadians();
    swerve.drive(new ChassisSpeeds(0, 0, 0), speakerAngle);
  }
}
