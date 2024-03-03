package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.AMP_ANGLE;
import static frc.robot.Constants.ShooterConstants.AMP_SPEED;
import static frc.robot.Constants.ShooterConstants.DEFAULT_SPEED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ScoringState;
import frc.robot.ScoringState.GoalMode;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class PrepareToShoot extends Command {
  Shooter shooter;
  Swerve swerve;
  Arm arm;

  public PrepareToShoot(Shooter shooter, Swerve swerve, Arm arm) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.arm = arm;
    addRequirements(shooter, arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch (ScoringState.goalMode) {
      case AMP:
        shooter.setTopSpeed(-AMP_SPEED);
        shooter.setBottomSpeed(AMP_SPEED);
        arm.setAngle(AMP_ANGLE);
        break;
      case SPEAKER:
        shooter.setSpeed(DEFAULT_SPEED);
        arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
        break;
      case STAGE:
        // shooter.setSpeed(DEFAULT_SPEED);
        // arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
        break;
    }  }

  @Override
  public boolean isFinished() {
    double shooterAngle = ScoringState.goalMode == GoalMode.SPEAKER ? ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()): AMP_ANGLE;
    return shooter.atSpeed() && swerve.atSpeakerYaw() && MathUtil.isNear(arm.getAngle().getRadians(), shooterAngle, 0.01); // TODO: FIX THE SETPOINT THING
  }

}
