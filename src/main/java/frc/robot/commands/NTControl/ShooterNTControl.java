package frc.robot.commands.NTControl;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

/* this really coulda been like two lines but now its a command */
public class ShooterNTControl extends Command {

  Shooter shooter;

  public ShooterNTControl(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setSpeed(0);
    SmartDashboard.putNumber("Shooter/manualSpeed", 0);
  }

  @Override
  public void execute() {
    shooter.setSpeed(SmartDashboard.getNumber("Shooter/manualSpeed", 0));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
  }
}
