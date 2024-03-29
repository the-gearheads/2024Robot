// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class CustomProxy extends Command {
  private Supplier<Command> supplier;
  private Command command;

  public CustomProxy(Supplier<Command> supplier, Subsystem... requirements) {
    this.supplier = supplier;
    addRequirements(requirements);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.command = supplier.get();
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }

}
