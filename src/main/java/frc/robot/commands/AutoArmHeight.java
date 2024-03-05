package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
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
    arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
  }
}
