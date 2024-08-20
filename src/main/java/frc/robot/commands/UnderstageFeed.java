package frc.robot.commands;

import static frc.robot.Constants.ArmConstants.NOTE_FEEDING_ANGLE_TOLERANCE;
import static frc.robot.Constants.ArmConstants.NOTE_UNDERSTAGE_FEEDING_ANGLE;
import static frc.robot.Constants.ShooterConstants.DEFAULT_SPEED;
import static frc.robot.Constants.SwerveConstants.NOTE_FEEDING_YAW_TOLERANCE;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class UnderstageFeed extends Command {
  Arm arm;
  Feeder feeder;
  Shooter shooter;
  Swerve swerve;
  boolean waitForYaw;
  
  public UnderstageFeed(Arm arm, Feeder feeder, Shooter shooter, Swerve swerve, boolean waitForYaw) {
    this.arm = arm;
    this.feeder = feeder;
    this.shooter = shooter;
    this.swerve = swerve;
    this.waitForYaw = waitForYaw;
    addRequirements(shooter, feeder, arm);
  }

  @Override
  public void execute() {
    Translation2d pose = swerve.getPose().getTranslation();
    arm.setAngle(NOTE_UNDERSTAGE_FEEDING_ANGLE);
    shooter.setSpeed(DEFAULT_SPEED);
    boolean atYaw = swerve.atYaw(ShooterCalculations.getYaw(pose).getRadians(), NOTE_FEEDING_YAW_TOLERANCE);
    boolean atYawIfWaiting = waitForYaw ? atYaw : true;

    Logger.recordOutput("UnderstageFeed/AtYaw", atYawIfWaiting);
    Logger.recordOutput("UnderstageFeed/WaitForYaw", waitForYaw);

    if (arm.atPoint(NOTE_UNDERSTAGE_FEEDING_ANGLE, NOTE_FEEDING_ANGLE_TOLERANCE) && shooter.atSpeed() && atYawIfWaiting) {
        feeder.run();
    };
  }
}
