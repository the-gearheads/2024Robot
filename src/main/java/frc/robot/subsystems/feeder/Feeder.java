package frc.robot.subsystems.feeder;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.FlywheelMotor;

import static frc.robot.Constants.FeederConstants.*;

import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  public FlywheelMotor feederMotor = new FlywheelMotor("Feeder", FEEDER_ID, PID, FEEDER_FF, true, true);
  public FlywheelMotor handoffMotor = new FlywheelMotor("Handoff", HANDOFF_ID, PID, FEEDER_FF, true, true);
  Trigger noteInPlaceSwitch = new Trigger(new DigitalInput(NOTE_SWITCH_ID)::get).negate().debounce(0.02);

  public Feeder() {
    SmartDashboard.putNumber("Feeder/RunSpeed", SPEED);
  }

  public void periodic() {
    feederMotor.periodic();
    feederMotor.log();
    handoffMotor.periodic();
    handoffMotor.log();
    Logger.recordOutput("Feeder/NoteSwitch", noteInPlaceSwitch.getAsBoolean());
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
    return Commands.run(this::run, this).andThen(new WaitCommand(seconds)).andThen(Commands.run(this::stop, this));
  }

  public Trigger getNoteSwitch() {
    return noteInPlaceSwitch;
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
