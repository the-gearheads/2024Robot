package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.leds.Leds;

public class LedControl extends Command {
  Leds leds;
  Feeder feeder;

  public LedControl(Leds leds, Feeder feeder) {
    this.leds = leds;
    this.feeder = feeder;
    addRequirements(leds);
  }

  @Override
  public void execute() {
    if (feeder.getBeamBreakSwitch().getAsBoolean()) {
      leds.setState(LedState.FLASH_NOTE);
    } else {
      leds.setState(LedState.BLACK);
    }
  }
}
