package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

/* this really coulda been like two lines but now its a command */
public class ArmNTControl extends Command {

  Arm arm;

  public ArmNTControl(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Arm/manualAngle", arm.getAngle().getDegrees());
  }

  @Override
  public void execute() {
    arm.setAngle(Units.degreesToRadians(SmartDashboard.getNumber("Arm/manualAngle", arm.getAngle().getDegrees())));
  }

  @Override
  public void end(boolean interrupted) {
  }
}
