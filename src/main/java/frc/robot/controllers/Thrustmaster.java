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

  public Trigger getBabyBird() {
    return new Trigger(()->{return joy.getPOV() == 180;});
  }

  // scoring modes, controls teleop controls, 3 buttons on top of thrust stick
  public Trigger getSetAmpModeBtn() {
    return new Trigger(()->joy.getRawButton(3));
  }
  
  public Trigger getSetSpeakerModeBtn() {
    return new Trigger(()->joy.getRawButton(4));
  }
  
  public Trigger getSetStageModeBtn() {
    return new Trigger(()->joy.getRawButton(2));
  }

  // overrides on left button face
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

  // climber controlled by two left btns, to adjust left vs right climber the thrust stick axis left and right is used.
  public  Trigger climberDown() {
    return new Trigger(()->joy.getRawButton(9));
  }

  public  Trigger climberUp() {
    return new Trigger(()->joy.getRawButton(6));
  }

  public double getClimberProportion() {
    return Controllers.deadband(joy.getX());
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

  public Trigger getAutoClimb() {
    return new Trigger(()->{return joy.getPOV() == 270;});
  }
}
