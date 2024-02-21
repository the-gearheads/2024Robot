// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoShooter;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.commands.Teleop;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.Constants.ArmConstants.armOverrideVoltage;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
    arm.setDefaultCommand(Commands.run(()->{
     arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
    }, arm));
    // arm.setDefaultCommand(new ArmNTControl(arm));

    shooter.setDefaultCommand(new AutoShooter(shooter, swerve, feeder));

    feeder.setDefaultCommand(Commands.run(feeder::stop, feeder));
    intake.setDefaultCommand(Commands.run(intake::stop, intake));

    sysidAuto.addSysidRoutine(shooter.getSysIdRoutine(), "Shooter");
    sysidAuto.addSysidRoutine(swerve.getSysIdRoutine(), "Swerve");
    sysidAuto.addSysidRoutine(swerve.getSysIdRoutineSteer(), "SwerveSteer");
    sysidAuto.addSysidRoutine(swerve.getSysIdRoutineAngular(), "SwerveAngular");
    sysidAuto.addSysidRoutine(arm.getSysIdRoutine(), "Arm");
    sysidAuto.addSysidRoutine(intake.getSysIdRoutine(), "Intake");
    sysidAuto.addSysidRoutine(feeder.getSysIdRoutine(), "Feeder");

    NamedCommands.registerCommand("ShooterStart", Commands.run(()->{
      shooter.setSpeed(Constants.ShooterConstants.DEFAULT_SPEED);
    }, shooter));

    NamedCommands.registerCommand("ShooterWaitForSpeed", Commands.waitUntil(shooter::atSpeed));
    NamedCommands.registerCommand("ShooterStop", Commands.run(()->{
      shooter.setSpeed(0);
    }, shooter));

    NamedCommands.registerCommand("FeederStart", Commands.run(feeder::run, feeder));
    NamedCommands.registerCommand("FeederStop", Commands.run(feeder::stop, feeder));
    NamedCommands.registerCommand("IntakeStart", Commands.run(intake::run, intake));
    NamedCommands.registerCommand("IntakeStop", Commands.run(intake::stop, intake));

    NamedCommands.registerCommand("IntakeNote", new IntakeNote(feeder, intake));
    NamedCommands.registerCommand("ShootWhenReady", new Shoot(shooter, feeder, swerve, arm));

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
      Rotation2d rot = new Rotation2d(180);
      if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) rot = new Rotation2d(Math.PI);
      swerve.resetPose(new Pose2d(swerve.getPose().getTranslation(), rot));
    }));

    Controllers.driverController.getResetPoseButton().onTrue(new InstantCommand(() -> {
        Pose2d pose = new Pose2d(new Translation2d(Units.inchesToMeters(36.2 + (29.875 / 2.0)), Units.inchesToMeters(218.4)), Rotation2d.fromDegrees(180));
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) pose = GeometryUtil.flipFieldPose(pose);
        swerve.resetPose(pose);
    }));

    Controllers.driverController.getPatthfindButton().onTrue(new ProxyCommand(()->{
      return swerve.pathFindTo(swerve.getPose().plus(new Transform2d(new Translation2d(1, 1), swerve.getPose().getRotation()))); // MUST be at least 6 bc of size of blocks in minecraft
    }));

    Controllers.driverController.getShootButton().whileTrue(new Shoot(shooter, feeder, swerve, arm));

    Controllers.operatorController.getIntakeNote().whileTrue(
      new IntakeNote(feeder, intake)
    );

    Controllers.operatorController.getShooterOverride().whileTrue(Commands.run(() -> {
       shooter.setSpeed(Constants.ShooterConstants.DEFAULT_SPEED);
      },
      shooter
    ));

    Controllers.operatorController.getShooterRevOverride().whileTrue(Commands.run(() -> {
       shooter.setSpeed(-Constants.ShooterConstants.DEFAULT_SPEED);
      },
      shooter
    ));

    Controllers.operatorController.getArmUp().whileTrue(Commands.run(()->{
      arm.setVoltage(armOverrideVoltage);
      arm.runPid = false;
      arm.resetToCurrentPose();
    }));

    Controllers.operatorController.getArmDown().whileTrue(Commands.run(()->{
      arm.setVoltage(armOverrideVoltage.negate());
      arm.runPid = false;
      arm.resetToCurrentPose();
    }));

    Controllers.operatorController.getArmAutosOff().onTrue(new InstantCommand(()->{
      arm.setDefaultCommand(Commands.run(()->{}, arm));
      shooter.setDefaultCommand(Commands.run(()->{shooter.setSpeed(0);}, shooter));
      Commands.run(()->{}, shooter).schedule();
      Commands.run(()->{}, arm).schedule();
      shooter.setSpeed(0);
    }));

    Controllers.operatorController.getArmAutosOn().onTrue(new InstantCommand(()->{
    arm.setDefaultCommand(Commands.run(()->{
     arm.setAngle(ShooterCalculations.getShooterAngle(swerve.getPose().getTranslation()));
    }, arm));
    shooter.setDefaultCommand(new AutoShooter(shooter, swerve, feeder));
    }));

    Controllers.operatorController.getIntakeOverride().whileTrue(Commands.startEnd(
      intake::run,
      intake::stop,
      intake
    ));

    Controllers.operatorController.getIntakeRevOverride().whileTrue(Commands.startEnd(
      intake::runReverse,
      intake::stop,
      intake
    ));


    Controllers.operatorController.getFeederOverride().whileTrue(Commands.startEnd(
      feeder::run,
      feeder::stop,
      feeder
    ));

    Controllers.operatorController.getFeederRevOverride().whileTrue(Commands.startEnd(
      feeder::runReverse,
      feeder::stop,
      feeder
    ));

    Controllers.operatorController.getAmpOverride().whileTrue(Commands.run(
      ()->{
        shooter.setTopSpeed(-Constants.ShooterConstants.AMP_SPEED);
        shooter.setBottomSpeed(Constants.ShooterConstants.AMP_SPEED);
      },
      shooter
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
