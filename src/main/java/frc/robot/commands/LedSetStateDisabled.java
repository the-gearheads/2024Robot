package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.leds.Leds;

public class LedSetStateDisabled extends Command {
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  //TODO: move these at the top of the class
  LedState desiredState;
  Leds leds;

  public LedSetStateDisabled(Leds leds, LedState state) {
    addRequirements(leds);
    desiredState = state;
    this.leds = leds;
  }

  @Override
  public void execute() {
    leds.setState(desiredState);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    leds.setState(leds.defaultState);
  }
}
