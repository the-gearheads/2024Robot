// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmNTControl;
import frc.robot.commands.FeederNTControl;
import frc.robot.commands.IntakeNTControl;
import frc.robot.commands.Teleop;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls)s. Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve swerve = new Swerve();
  public final Leds leds = new Leds();
  private final Shooter shooter = new Shooter();
  public final Arm arm = new Arm();
  public final Feeder feeder = new Feeder();
  public final Intake intake = new Intake();
  private final SysidAutoPicker sysidAuto = new SysidAutoPicker();
  private SendableChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    updateControllers();
    swerve.setDefaultCommand(new Teleop(swerve));
    // arm.setDefaultCommand(Commands.run(()->{
    //  arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
    // }, arm));
    arm.setDefaultCommand(new ArmNTControl(arm));
    feeder.setDefaultCommand(new FeederNTControl(feeder));
    intake.setDefaultCommand(new IntakeNTControl(intake));

    sysidAuto.addSysidRoutine(shooter.getSysIdRoutine(), "Shooter");
    sysidAuto.addSysidRoutine(swerve.getSysIdRoutine(), "Swerve");
    sysidAuto.addSysidRoutine(swerve.getSysIdRoutineSteer(), "SwerveSteer");
    sysidAuto.addSysidRoutine(swerve.getSysIdRoutineAngular(), "SwerveAngular");
    sysidAuto.addSysidRoutine(arm.getSysIdRoutine(), "Arm");
    sysidAuto.addSysidRoutine(intake.getSysIdRoutine(), "Intake");
    sysidAuto.addSysidRoutine(feeder.getSysIdRoutine(), "Feeder");

    // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
 
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void updateControllers() {
    if (!Controllers.didControllersChange())
      return;
    System.out.println("Updating controller layout");

    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    Controllers.driverController.getGyroZeroButton().onTrue(new InstantCommand(() -> {
      Rotation2d rot = new Rotation2d(0);
      if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) rot = new Rotation2d(Math.PI);
      swerve.resetPose(new Pose2d(swerve.getPose().getTranslation(), rot));
    }));

    Controllers.driverController.getResetPoseButton().onTrue(new InstantCommand(() -> {
        Pose2d pose = new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(0));
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) pose = GeometryUtil.flipFieldPose(pose);
        swerve.resetPose(pose);
    }));

    Controllers.driverController.getPatthfindButton().onTrue(new ProxyCommand(()->{
      return swerve.pathFindTo(swerve.getPose().plus(new Transform2d(new Translation2d(1, 1), swerve.getPose().getRotation()))); // MUST be at least 6 bc of size of blocks in minecraft
    }));

    Controllers.driverController.getIntake().whileTrue(Commands.startEnd(
      intake::run,
      intake::stop,
      intake
    ));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */ 
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return sysidAuto.get();
    // return autoChooser.getSelected();
  }
}
