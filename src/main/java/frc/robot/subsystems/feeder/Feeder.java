package frc.robot.subsystems.feeder;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.FlywheelMotor;

import static frc.robot.Constants.FeederConstants.*;

import java.util.concurrent.atomic.AtomicBoolean;

import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  public FlywheelMotor feederMotor = new FlywheelMotor("Feeder", FEEDER_ID, PID, FEEDER_FF, true, true);
  public FlywheelMotor handoffMotor = new FlywheelMotor("Handoff", HANDOFF_ID, PID, FEEDER_FF, true, true);
  private DigitalInput beamBreak = new DigitalInput(BEAMBREAK_SWITCH_ID);
  private Trigger beamBreakSwitch = new Trigger(Robot.triggerEventLoop, ()->!beamBreak.get());

  // There's a note in the bot when low, so we only care abt the falling edge (that being said, this should be configued to only act on the falling edge)
  // After reading the implementation, it seems the callback runs in another thread
  // Whether the interrupt should stop the feeder when a note hits it.
  AtomicBoolean shouldStopFeeder = new AtomicBoolean();
  // A "callback" for the command so it knows that the feeder was stopped by the interrupt
  AtomicBoolean didStopFeeder = new AtomicBoolean();
  // Problem: this doesn't debounce. I don't know how to make it (at least in a trivial way).
  AsynchronousInterrupt bbInterrupt = new AsynchronousInterrupt(beamBreak, (risingEdge, fallingEdge) -> {
    if(shouldStopFeeder.get() && fallingEdge) {
      runAtSpeed(0);
      // Need to run periodic so the pid controllers can run and do their thing, since we want to stop NOW.
      feederMotor.periodic();
      handoffMotor.periodic();
      didStopFeeder.set(true);
      shouldStopFeeder.set(false);
    }
  });

  public Feeder() {
    SmartDashboard.putNumber("Feeder/RunSpeed", SPEED);
    bbInterrupt.setInterruptEdges(false, true);
  }

  public void periodic() {
    feederMotor.periodic();
    feederMotor.log();
    handoffMotor.periodic();
    handoffMotor.log();

    Logger.recordOutput("Feeder/beamBreakSwitch", beamBreakSwitch.getAsBoolean());
    Logger.recordOutput("Feeder/ShouldStopFeeder", shouldStopFeeder.get());
    Logger.recordOutput("Feeder/DidStopFeeder", didStopFeeder.get());
  }
  
  public void run() {
    runAtSpeed(SmartDashboard.getNumber("Feeder/RunSpeed", SPEED));
  }

  public void runReverse() {
    runAtSpeed(-SmartDashboard.getNumber("Feeder/RunSpeed", SPEED));
  }

  public void runAtSpeed(double speed) {
    feederMotor.setSpeed(speed);
    handoffMotor.setSpeed(speed);
  }

  public void stop() {
    feederMotor.setSpeed(0);
    handoffMotor.setSpeed(0);
  }

  public Command getRunFeederCommand(double seconds) {
    return this.run(this::run)
      .andThen(new WaitCommand(seconds))
      .andThen(Commands.run(this::stop, this))
      .withName("RunFeederCommand" + seconds + "Seconds");
  }

  public Command getRunFeederCommand() {
    return this.runOnce(()->{
      shouldStopFeeder.set(true);
      didStopFeeder.set(false);
    }).andThen(
      this.run(this::run).until(didStopFeeder::get)
    ).withName("RunFeederCommandAsync");
  }

  public Trigger getBeamBreakSwitch() {
    return beamBreakSwitch;
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, 
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          feederMotor.setVolts(volts);
          handoffMotor.setVolts(volts);
        },
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
  }

}
