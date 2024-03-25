package frc.robot.commands;

import static frc.robot.Constants.ArmConstants.NOTE_FEEDING_ANGLE;
import static frc.robot.Constants.ArmConstants.NOTE_FEEDING_ANGLE_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.NOTE_FEEDING_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFeederNote extends Command {
  Arm arm;
  Feeder feeder;
  Shooter shooter;

  public ShootFeederNote(Arm arm, Feeder feeder, Shooter shooter) {
    this.arm = arm;
    this.feeder = feeder;
    this.shooter = shooter;
    addRequirements(shooter, feeder, arm);
  }

  @Override
  public void execute() {
    arm.setAngle(NOTE_FEEDING_ANGLE);
    shooter.setSpeed(NOTE_FEEDING_SPEED);
    if (arm.atPoint(NOTE_FEEDING_ANGLE, NOTE_FEEDING_ANGLE_TOLERANCE) && shooter.atSpeed()) {
      feeder.run();
    };
  }
}
