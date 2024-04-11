package frc.robot.commands.NTControl;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

/* this really coulda been like two lines but now its a command */
public class FeederNTControl extends Command {

  Feeder feeder;

  public FeederNTControl(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Feeder/manualRun", false);//TODO: move string into const
  }

  @Override
  public void execute() {
    if(SmartDashboard.getBoolean("Feeder/manualRun", false)) {//TODO: move string into const
      feeder.run();
    } else {
      feeder.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }
}
