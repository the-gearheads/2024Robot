package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

public class EndIfNoNote extends Command {

  Feeder feeder;

  public EndIfNoNote(Feeder feeder) {
    this.feeder = feeder;
  }

  public double timeStart = Timer.getFPGATimestamp();

  @Override
  public void initialize() {
    timeStart = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    System.out.println("waitinggggggggg, feeder thinks it has a note: " + feeder.getBeamBreakSwitch().getAsBoolean() + " time: " + (Timer.getFPGATimestamp() - timeStart) + " start: " + timeStart);
  }

  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - timeStart) > 2) && (feeder.getBeamBreakSwitch().getAsBoolean() == false);
  }

  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("Ended becauase no note", false);
  }
}
