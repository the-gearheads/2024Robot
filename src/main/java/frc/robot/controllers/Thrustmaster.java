package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Thrustmaster implements OperatorController {
  Joystick joy;
  public Thrustmaster(int id) {
    joy = new Joystick(id);
  }
  
  // https://ts.thrustmaster.com/download/accessories/Manuals/T16000M/T16000M-User_manual.pdf for button layout

  public Trigger getIntakeNote() {
    return new Trigger(()->joy.getRawButton(1));
  }

  public Trigger getShooterUp() {
    return new Trigger(() -> joy.getRawButton(5));
  }
  
  public Trigger getShooterDown() {
    return new Trigger(() -> joy.getRawButton(10));
  }
  
  public Trigger getShooterAutoToggle() {
    return new Trigger(() -> joy.getRawButton(8));
  }

  public Trigger getAmpOverride() {
    return new Trigger(()->joy.getRawButton(3));
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

}
