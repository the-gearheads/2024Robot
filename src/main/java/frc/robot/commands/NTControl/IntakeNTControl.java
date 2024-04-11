package frc.robot.commands.NTControl;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/* this really coulda been like two lines but now its a command */
public class IntakeNTControl extends Command {

  Intake feeder; //TODO: change name from feeder to intake

  public IntakeNTControl(Intake intake) {
    this.feeder = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Intake/manualRun", false);//TODO: move string into const
  }

  @Override
  public void execute() {
    if(SmartDashboard.getBoolean("Intake/manualRun", false)) {//TODO: move string into const
      feeder.run();
    } else {
      feeder.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }
}
