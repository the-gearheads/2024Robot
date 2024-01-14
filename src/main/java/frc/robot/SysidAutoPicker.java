package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SysidAutoPicker {
  private SendableChooser<Command> chooser = new SendableChooser<>();

  public SysidAutoPicker() {
    SmartDashboard.putData("SysidRoutines", chooser);
  }

  public void addSysidRoutine(SysIdRoutine routine, String name) {
    chooser.addOption(name + " Quasi Forward", routine.quasistatic(Direction.kForward));
    chooser.addOption(name + " Quasi Reverse", routine.quasistatic(Direction.kReverse));
    chooser.addOption(name + " Dynamic Forward", routine.dynamic(Direction.kForward));
    chooser.addOption(name + " Dynamic Reverse", routine.dynamic(Direction.kReverse));
  }

  public Command get() {
    return chooser.getSelected();
  }
}
