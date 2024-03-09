package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.AMP_ANGLE_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.AMP_SCORE_ANGLE;
import static frc.robot.Constants.ShooterConstants.AMP_WAIT_ANGLE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;

public class AmpScore extends Command {
  Feeder feeder;
  Arm arm;

  public AmpScore(Arm arm, Feeder feeder) {
    this.arm = arm;
    this.feeder = feeder;
    addRequirements(feeder, arm);
  }

  @Override
  public void execute() {
    arm.setAngle(AMP_SCORE_ANGLE);
    if (MathUtil.isNear(arm.getAngle().getRadians(), AMP_SCORE_ANGLE, AMP_ANGLE_TOLERANCE)) {
      feeder.run();
    }
  }

  @Override 
  public void end(boolean interrupted) {
    arm.setAngle(AMP_WAIT_ANGLE);
  }
}
