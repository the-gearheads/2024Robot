package frc.robot.commands;


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
    ScoringState.babyBirdMode = false; // If this command is running we never want this //LAC: whatr happens if this is running? Would it be possible to create a check in here to see if it is running and then turn it back once the operation is complete?
    arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
  }
}
