package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ScoringState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.Polygon;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class AutoShooter extends Command {
  Shooter shooter;
  Swerve swerve;
  Feeder feeder;
  Polygon shooterSpin = new Polygon(SHOOTER_SPIN_X, SHOOTER_SPIN_Y);

  public AutoShooter(Shooter shooter, Swerve swerve, Feeder feeder) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.feeder = feeder;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d pose = swerve.getPoseAllianceRelative();
    if (shooterSpin.contains(pose) && feeder.getBeamBreakSwitch().getAsBoolean()) {
      switch (ScoringState.goalMode) {
        case SPEAKER:
        default:
          shooter.setSpeed(DEFAULT_SPEED);
          break;
        case AMP:
          shooter.setTopSpeed(-AMP_SPEED);
          shooter.setBottomSpeed(AMP_SPEED);
          break;
      }
    } else {
      shooter.setSpeed(0);
    }
  }
}
