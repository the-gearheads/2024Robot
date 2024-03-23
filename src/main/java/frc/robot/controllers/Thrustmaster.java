package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Thrustmaster implements OperatorController {
  Joystick joy;
  public Thrustmaster(int id) {
    joy = new Joystick(id);
  }
  
  // https://ts.thrustmaster.com/download/accessories/Manuals/T16000M/T16000M-User_manual.pdf for button layout

  public Trigger getIntakeNote() {
    return new Trigger(()->joy.getRawButton(1));
  }

  public Trigger getArmUp() {
    return new Trigger(() -> joy.getRawButton(5));
  }
  
  public Trigger getArmDown() {
    return new Trigger(() -> joy.getRawButton(10));
  }
  
  public Trigger getArmAutosOff() {
    return new Trigger(() -> joy.getRawButton(8));
  }
  
  public Trigger getArmAutosOn() {
    return new Trigger(() -> joy.getRawButton(7));
  }

  // right side manual controls shooter -> feeeder -> intake left to right, top forwards, bottom backwards
  public Trigger getShooterOverride() {
    return new Trigger(()->joy.getRawButton(13));
  }

  public Trigger getShooterRevOverride() {
    return new Trigger(()->joy.getRawButton(14));
  }

  public Trigger getFeederOverride() {
    return new Trigger(()->joy.getRawButton(12));
  }

  public Trigger getFeederRevOverride() {
    return new Trigger(()->joy.getRawButton(15));
  }
  
  public Trigger getIntakeOverride() {
    return new Trigger(()->joy.getRawButton(11));
  }
  
  public Trigger getIntakeRevOverride() {
    return new Trigger(()->joy.getRawButton(16));
  }

  public Trigger getSetAmpModeBtn() {
    return new Trigger(()->joy.getRawButton(3));
  }
  
  public Trigger getSetSpeakerModeBtn() {
    return new Trigger(()->joy.getRawButton(4));
  }
  
  public Trigger getSetStageModeBtn() {
    return new Trigger(()->joy.getRawButton(2));
  }

  public  Trigger climberDown() {
    return new Trigger(()->joy.getRawButton(9));
  }

  public  Trigger climberUp() {
    return new Trigger(()->joy.getRawButton(6));
  }

  public double getClimberProportion() {
    return Controllers.deadband(joy.getY());
  }
}
