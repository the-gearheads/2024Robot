package frc.robot.commands;

import static frc.robot.Constants.ArmConstants.NOTE_FEEDING_ANGLE;
import static frc.robot.Constants.ArmConstants.NOTE_FEEDING_ANGLE_TOLERANCE;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFeederNote extends Command {
  Arm arm;
  Feeder feeder;
  Shooter shooter;
  Supplier<Pose2d> poseSupplier;

  public ShootFeederNote(Arm arm, Feeder feeder, Shooter shooter, Supplier<Pose2d> poseSupplier) {
    this.arm = arm;
    this.feeder = feeder;
    this.shooter = shooter;
    this.poseSupplier = poseSupplier;
    addRequirements(shooter, feeder, arm);
  }

  @Override
  public void execute() {
    arm.setAngle(NOTE_FEEDING_ANGLE);
    shooter.setSpeed(ShooterCalculations.getFeedRpm(poseSupplier.get().getTranslation()));
    if (arm.atPoint(NOTE_FEEDING_ANGLE, NOTE_FEEDING_ANGLE_TOLERANCE) && shooter.atSpeed()) {
      feeder.run();
    };
  }
}
