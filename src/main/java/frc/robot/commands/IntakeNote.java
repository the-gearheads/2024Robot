package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

public class IntakeNote extends Command {
  Feeder feeder;
  Intake intake;
  boolean stopAtEnd;

  public IntakeNote(Feeder feeeder, Intake intake, boolean stopAtEnd) {
    this.feeder = feeeder;
    this.intake = intake;
    this.stopAtEnd = stopAtEnd;
    addRequirements(feeder, intake);
  }

  public IntakeNote(Feeder feeder, Intake intake) {
    this(feeder, intake, true);
  }

  @Override
  public void initialize() {
    intake.run();
    feeder.run();
  }

  @Override
  public boolean isFinished() {
    return feeder.getBeamBreakSwitch().getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    if(stopAtEnd) {
      feeder.stop();
      intake.stop();
    }
  }
}
