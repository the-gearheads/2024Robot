package frc.robot.subsystems.leds;


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import static frc.robot.Constants.Leds.*;

public class Leds extends SubsystemBase {
  /** Creates a new LEDS. */
  private AddressableLED ledStrip;
  private AddressableLEDSim ledSim;

  private AddressableLEDBuffer buffer;

  // LED States
  private LedState state;
  // Assuming that both strips are the same length - therefore we can use one buffer

  public LedState defaultState = LedState.BLACK;

  public Leds() {
    buffer = new AddressableLEDBuffer(LENGTH);

    ledStrip = new AddressableLED(PORT);
    ledStrip.setLength(buffer.getLength());

    ledSim = new AddressableLEDSim(ledStrip);
    ledSim.setOutputPort(PORT);

    state = defaultState;

    // send voltages to the strips
    startLED();
  }

  public void setState(LedState newState) {
    this.state = newState;
  }

  public void resetState() {
    this.state = defaultState;
  }

  public Command getSetStateCommand(LedState newState) {
    return Commands.run(() -> setState(newState), this);
  }

  public Command setStateForTimeCommand(LedState newState, double waitSecs) {
    return getSetStateCommand(newState).withTimeout(waitSecs);
  }

  // method to flush both strips
  public void startLED() {
    ledStrip.start();
  }

  @Override
  public void periodic() {

    if(DriverStation.isDisabled()) {
      defaultState = LedState.RAINBOW;
    } else {
      defaultState = LedState.BLACK;
    }

    this.state.updateBuffer(buffer);

    /* Should overwrite what the above wrote to the buffer for the out-of-time warning */
    if (DriverStation.isTeleop()) {
      double timeRemaining = Robot.matchTime;
      if (timeRemaining >= 57.0 && timeRemaining < 60.0) {
        LedState.FLASH_GREEN.updateBuffer(buffer);
      } else if (timeRemaining >= 23.0 && timeRemaining < 26.0) {
        LedState.FLASH_YELLOW.updateBuffer(buffer);
      } else if (timeRemaining >= 5.0 && timeRemaining < 8.0) {
        LedState.FLASH_RED.updateBuffer(buffer);
      }
    }
    this.ledStrip.setData(buffer);

    state = defaultState; // reset state to default if not set by a command
  }
}

