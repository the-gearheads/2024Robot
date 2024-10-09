// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Teleop;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.GPDetect;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls)s. Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve swerve = new Swerve();
  public final GPDetect gpDetect = new GPDetect(swerve);
  public final Leds leds = new Leds();
  private final SysidAutoPicker sysidAuto = new SysidAutoPicker();
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    updateControllers();
    swerve.setDefaultCommand(new Teleop(swerve, gpDetect));
    // arm.setDefaultCommand(new ArmNTControl(arm));

    sysidAuto.addSysidRoutine(swerve.getSysIdRoutine(), "Swerve");
    sysidAuto.addSysidRoutine(swerve.getSysIdRoutineSteer(), "SwerveSteer");
    sysidAuto.addSysidRoutine(swerve.getSysIdRoutineAngular(), "SwerveAngular");

    NamedCommands.registerCommand("ShooterStart", Commands.none());

    NamedCommands.registerCommand("ShooterWaitForSpeed", Commands.none());
    NamedCommands.registerCommand("ShooterStop", Commands.none());

    NamedCommands.registerCommand("FeederStart", Commands.none());
    NamedCommands.registerCommand("FeederStop", Commands.none());
    NamedCommands.registerCommand("IntakeStart", Commands.none());
    NamedCommands.registerCommand("IntakeStop", Commands.none());
    
    NamedCommands.registerCommand("IntakeNote", Commands.none());
    NamedCommands.registerCommand("PrepareShoot", Commands.none());
    NamedCommands.registerCommand("AutonAutoArmHeight", Commands.none());
    NamedCommands.registerCommand("AutoArmHeight", Commands.none());
    NamedCommands.registerCommand("ShootWhenReady", Commands.none());
    NamedCommands.registerCommand("ShootPreloadWhenReady", Commands.none());
    NamedCommands.registerCommand("FeedNote", Commands.none());
    NamedCommands.registerCommand("WaitForShot", Commands.none());
    NamedCommands.registerCommand("AlignToSpeakerYaw", Commands.none());
    NamedCommands.registerCommand("IntakeAndShoot", Commands.none());
    NamedCommands.registerCommand("Sweep", Commands.none());


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("DisableVision", new InstantCommand(() -> {
      swerve.disableVision();
    }));
    NamedCommands.registerCommand("EnableVision", new InstantCommand(() -> {
      swerve.enableVision();
    }));
  }

  public void setAllBrakeCoast(boolean willBrake) {
    swerve.setBrakeCoast(willBrake);
  }

  public void updateControllers() {
    if (!Controllers.didControllersChange())
      return;
    System.out.println("Updating controller layout");

    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    // teleop controlls
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */ 
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return sysidAuto.get();
    return autoChooser.getSelected();
  }
}
