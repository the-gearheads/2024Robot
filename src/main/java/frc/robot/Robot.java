// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.LedSetStateDisabled;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.leds.LedState;
import frc.robot.util.TriConsumer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /* Globals :( */
  public static double matchTime = -1;
  private static double matchTimeStart = 0;

  public static final EventLoop triggerEventLoop = new EventLoop();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  /* Dead code warnings */
  @SuppressWarnings("all")
  public void robotInit() {
    Logger.recordMetadata("Build Date", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("Git Branch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Git Dirty", BuildConstants.DIRTY == 1 ? "true" : "false");

    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

    Logger.registerURCL(URCL.startExternal());
    Logger.start();

    /* Log all commands running, both uniquely and by name. */
    Map<String, Integer> commandCounts = new HashMap<>();
    TriConsumer<Command, Boolean, String> logCommandFunction = (Command command, Boolean active, String reason) -> {
      String name = command.getName();
      int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
      commandCounts.put(name, count);
      final boolean[] isDefault = {false}; // I hate copilot for suggesting this as a workaround but it does in fact work
      String reqs = command.getRequirements()
        .stream()
        .map(subsystem -> {
          if(subsystem.getDefaultCommand() == command) {
            isDefault[0] = true;
          }
          return subsystem.getName();
        })
        .collect(Collectors.joining("_"));
      Logger.recordOutput("RunningCommands/Unique/" + name + (isDefault[0] ? "_DEFAULT_" : "_") + reqs + "_" + Integer.toHexString(command.hashCode()), reason);
      Logger.recordOutput("RunningCommands/All/" + name, count > 0);
    };

    CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
      logCommandFunction.accept(command, true, "RUNNING");
    });

    CommandScheduler.getInstance().onCommandFinish((Command command) -> {
      logCommandFunction.accept(command, false, "FINISHED");
    });

    CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
        logCommandFunction.accept(command, false, "INTERRUPTED");
    });

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    RobotController.setBrownoutVoltage(6);
    PathfindingCommand.warmupCommand().schedule();
    FollowPathCommand.warmupCommand().schedule();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    triggerEventLoop.poll();
    CommandScheduler.getInstance().run();
    m_robotContainer.updateControllers();
    ScoringState.periodic();

    if (DriverStation.isEStopped()) {
      m_robotContainer.leds.setState(LedState.HOT_PINK);
    }
  }

  private DigitalInput brakeCoastButton = new DigitalInput(Constants.BrakeCoastButton.PORT);
  private Debouncer brakeCoastButtonDebouncer = new Debouncer(0.05);
  private boolean lastBrakeCoastButton = false;
  private boolean isBraken = true;
  private boolean rumbled = true;


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    boolean output = brakeCoastButtonDebouncer.calculate(!brakeCoastButton.get());
    // m_robotContainer.leds.setState(LedState.RAINBOW);
    if(output && !lastBrakeCoastButton) {
      isBraken = !isBraken;
      m_robotContainer.setAllBrakeCoast(isBraken);
      new LedSetStateDisabled(m_robotContainer.leds, isBraken ? LedState.FLASH_RED : LedState.FLASH_GREEN).withTimeout(1).schedule();
      System.out.println("Brake/Coast: " + (isBraken ? "Brake" : "Coast"));
    }

    lastBrakeCoastButton = output;
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    matchTime = -1;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    CANSparkLowLevel.enableExternalUSBControl(false);
    m_robotContainer.setAllBrakeCoast(true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    matchTimeStart = Timer.getFPGATimestamp();
    m_robotContainer.setAllBrakeCoast(true);
    CANSparkLowLevel.enableExternalUSBControl(false);
    rumbled = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    matchTime = 135 - (Timer.getFPGATimestamp() - matchTimeStart);
    matchTime = matchTime < 0 ? 0 : matchTime;
    Logger.recordOutput("TeleopMatchTime", matchTime);
    if (matchTime < 30.2 && matchTime > 30 && !rumbled) {
      Controllers.driverController.getRumbleCommand(1.0, 0.2, 3).schedule();
      rumbled = true;
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CANSparkLowLevel.enableExternalUSBControl(true);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    List<Pose3d> notesPoses = SimulatedArena.getInstance().getGamePiecesByType("Note");
    Logger.recordOutput("FieldSimulation/NotesPositions", notesPoses.toArray(new Pose3d[0]));
  }
}
