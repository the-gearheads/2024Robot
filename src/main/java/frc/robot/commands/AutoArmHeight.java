package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.AMP_ANGLE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ScoringState;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.swerve.Swerve;

public class AutoArmHeight extends Command {
  Arm arm;
  Swerve swerve;

  public AutoArmHeight(Arm arm, Swerve swerve) {
    this.arm = arm;
    this.swerve = swerve;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    switch (ScoringState.goalMode) {
      case AMP:
        arm.setAngle(AMP_ANGLE);
        break;
      case SPEAKER:
        arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
        break;
      case STAGE:
        // shooter.setSpeed(DEFAULT_SPEED);
        // arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
        break;
    }
  }
}
