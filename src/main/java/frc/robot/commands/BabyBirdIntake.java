package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ScoringState;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;

public class BabyBirdIntake extends Command {
  Feeder feeder;
  Arm arm;
  Shooter shooter;
  boolean stopAtEnd;

  public BabyBirdIntake(Arm arm, Shooter shooter, Feeder feeder, boolean stopAtEnd) {
    this.feeder = feeder;
    this.arm = arm;
    this.shooter = shooter;
    this.stopAtEnd = stopAtEnd;
    addRequirements(feeder, arm, shooter);
  }

  public BabyBirdIntake(Arm arm, Shooter shooter, Feeder feeder) {
    this(arm, shooter, feeder, true);
  }

  @Override
  public void initialize() {
    feeder.runAtSpeed(-FeederConstants.SPEED);
    shooter.setSpeed(-ShooterConstants.DEFAULT_SPEED);
    arm.setAngle(ArmConstants.BABY_BIRD_ANGLE);
    ScoringState.babyBirdMode = true;
  }

  @Override
  public void execute() {
    ScoringState.babyBirdMode = true;
    ScoringState.feedBabyBirdMode();
  }

  @Override
  public boolean isFinished() {
    return feeder.getBeamBreakSwitch().getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    ScoringState.babyBirdMode = false;
    if(stopAtEnd) {
      feeder.stop();
    }
  }
}
