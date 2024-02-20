package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

public class IntakeNote extends Command {
  Feeder feeder;
  Intake intake;

  public IntakeNote(Feeder feeeder, Intake intake) {
    this.feeder = feeeder;
    this.intake = intake;
    addRequirements(feeder, intake);
  }

  @Override
  public void initialize() {
    intake.run();
    feeder.run();
  }

  @Override
  public boolean isFinished() {
    return feeder.getNoteSwitch().getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }
}
