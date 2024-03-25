// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ArmConstants.armOverrideVoltage;
import static frc.robot.Constants.FieldConstants.AMP_SCORE_POSE;
import static frc.robot.Constants.ShooterConstants.DEFAULT_SPEED;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoArmHeight;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.AutonAutoArmHeight;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.PrepareToShoot;
import frc.robot.commands.SwerveAlignToSpeaker;
import frc.robot.commands.Teleop;
import frc.robot.commands.NTControl.ArmNTControl;
import frc.robot.commands.NTControl.ShooterNTControl;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.MechanismViz;
import frc.robot.subsystems.NoteSimMgr;
import frc.robot.subsystems.NoteSimMgr.NoteState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.HandledSleep;

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
  public final Climber climber = new Climber();
  @SuppressWarnings("unused")
  private final MechanismViz mechanismViz = new MechanismViz(swerve, arm::getAngle, shooter.topMotor::getPosition, shooter.bottomMotor::getPosition, intake.motor::getPosition, feeder.feederMotor::getPosition, feeder.getBeamBreakSwitch()::getAsBoolean);
  @SuppressWarnings("unused")
  private final NoteSimMgr noteSimMgr = new NoteSimMgr(swerve::getPose, shooter.topMotor::getVelocity, shooter.bottomMotor::getVelocity, intake.motor::getVelocity, feeder.feederMotor::getVelocity);
  private final SysidAutoPicker sysidAuto = new SysidAutoPicker();
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    updateControllers();
    swerve.setDefaultCommand(new Teleop(swerve));
    // arm.setDefaultCommand(new AutoArmHeight(arm, swerve));
    arm.setDefaultCommand(new ArmNTControl(arm));

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
    sysidAuto.addSysidRoutine(climber.getSysIdRoutine(), "Climber");

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
    NamedCommands.registerCommand("PrepareShoot", new PrepareToShoot(shooter, swerve, arm));
    NamedCommands.registerCommand("AutonAutoArmHeight", new AutonAutoArmHeight(arm));
    NamedCommands.registerCommand("AutoArmHeight", new AutoArmHeight(arm, swerve));
    NamedCommands.registerCommand("ShootWhenReady", new PrepareToShoot(shooter, swerve, arm).andThen(feeder.getRunFeederCommand()));
    NamedCommands.registerCommand("FeedNote", new SequentialCommandGroup(
      new WaitUntilCommand(feeder.getBeamBreakSwitch()),
      feeder.getRunFeederCommand()
    ));
    NamedCommands.registerCommand("AlignToSpeakerYaw", new SwerveAlignToSpeaker(swerve));
    NamedCommands.registerCommand("IntakeAndShoot", Commands.run(()->{
      intake.run();
      feeder.run();
      shooter.setSpeed(DEFAULT_SPEED);
    }, feeder, intake, shooter));


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("DisableVision", new InstantCommand(() -> {
      swerve.disableVision();
    }));
    NamedCommands.registerCommand("EnableVision", new InstantCommand(() -> {
      swerve.enableVision();
    }));

    feeder.getBeamBreakSwitch().whileTrue(leds.setStateForTimeCommand(LedState.FLASH_LIME, 3));
  }

  public void setAllBrakeCoast(boolean willBrake) {
    arm.setBrakeCoast(willBrake);
    climber.setBrakeCoast(willBrake);
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
    Controllers.driverController.getAutoShootBtn().whileTrue(new PrepareToShoot(shooter, swerve, arm).andThen(Commands.run(feeder::run, feeder)));
    Controllers.driverController.getShootBtn().whileTrue(feeder.getRunFeederCommand());
    Controllers.driverController.getAlignBtn().whileTrue(new ProxyCommand(() -> {
      switch(ScoringState.goalMode) {
        case SPEAKER:
          return new PrepareToShoot(shooter, swerve, arm).repeatedly();
        case AMP:
          return new InstantCommand(); // will set forced angle to AMP because of ShooterCalculation getYaw being called in teleop
        case STAGE:
          return new InstantCommand(); // will set forced angle to stage bc shootercalculations getYaw in teleop
        default:
          return new InstantCommand();
      }
    }));

    Controllers.driverController.enableVision().onTrue(new InstantCommand(() -> swerve.enableVision()));
    Controllers.operatorController.getIntakeNote().whileTrue(new IntakeNote(feeder, intake));
    Controllers.driverController.getResetPoseBtn().onTrue(new InstantCommand(() -> {
        boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        Pose2d pos = AMP_SCORE_POSE;
        if(isRed) {
          pos = GeometryUtil.flipFieldPose(pos);
        }
        swerve.resetPose(pos);
    }));
    Controllers.driverController.getDisableVisionBtn().onTrue(new InstantCommand(() -> swerve.disableVision()));
    Controllers.operatorController.climberDown().whileTrue(Commands.run(() -> {
      climber.down(Controllers.operatorController.getClimberProportion());
    }, climber));
    Controllers.operatorController.climberUp().whileTrue(Commands.run(() -> {
      climber.up(Controllers.operatorController.getClimberProportion());
    }, climber));
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
      arm.getDefaultCommand().cancel();
      shooter.getDefaultCommand().cancel();
      arm.setDefaultCommand(new ArmNTControl(arm));
      shooter.setDefaultCommand(new ShooterNTControl(shooter));
      Commands.runOnce(()->{}, shooter).schedule();
      Commands.runOnce(()->{}, arm).schedule();
      shooter.setSpeed(0);
    }));

    Controllers.operatorController.getArmAutosOn().onTrue(new InstantCommand(()->{
      arm.getDefaultCommand().cancel();
      shooter.getDefaultCommand().cancel();
      arm.setDefaultCommand(new AutoArmHeight(arm, swerve));
      shooter.setDefaultCommand(new AutoShooter(shooter, swerve, feeder));
      Commands.runOnce(()->{}, shooter).schedule();
      Commands.runOnce(()->{}, arm).schedule();
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

    Controllers.operatorController.getSetSpeakerModeBtn().onTrue(new InstantCommand(()->{
      ScoringState.goalMode = ScoringState.GoalMode.SPEAKER;
    }));

    Controllers.operatorController.getSetAmpModeBtn().onTrue(new InstantCommand(()->{
      ScoringState.goalMode = ScoringState.GoalMode.AMP;
    }));

    Controllers.operatorController.getSetStageModeBtn().onTrue(new InstantCommand(()->{
      ScoringState.goalMode = ScoringState.GoalMode.STAGE;
    }));
  
    Controllers.driverController.reconfigureEverything().onTrue(new InstantCommand(()->{
      arm.configure();
      HandledSleep.sleep(100);
      shooter.topMotor.configure();
      HandledSleep.sleep(100);
      shooter.bottomMotor.configure();
      HandledSleep.sleep(100);
      feeder.feederMotor.configure();
      HandledSleep.sleep(100);
      feeder.handoffMotor.configure();
      HandledSleep.sleep(100);
      intake.motor.configure();
    }));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */ 
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return sysidAuto.get();
    if(Robot.isSimulation()) {
      noteSimMgr.setNoteState(NoteState.INTOOK);
    }
    ScoringState.goalMode = ScoringState.GoalMode.SPEAKER;
    return autoChooser.getSelected();
    // return sysidAuto.get();
  }
}
