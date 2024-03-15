package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.leds.Leds;

public class TeleopLedControl extends Command {
  Leds leds;
  Feeder feeder;

  public TeleopLedControl(Leds leds, Feeder feeder) {
    this.leds = leds;
    this.feeder = feeder;
    addRequirements(leds);
  }


  @Override
  public void initialize() {

  }
  @Override
  public void execute() {
    feeder.getBeamBreakSwitch().whileTrue(
      Commands.deadline(new WaitCommand(2), Commands.run(()->leds.setState(LedState.FLASH_LIME), leds))
    );
  }
}
