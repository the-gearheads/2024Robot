package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorController {
  public default Trigger getIntakeNote() {
    return new Trigger(()->false);
  }

  public default Trigger getShooterUp() {
    return new Trigger(()->false);
  }
  
  public default Trigger getShooterDown() {
    return new Trigger(()->false);
  }
  
  public default Trigger getShooterAutoToggle() {
    return new Trigger(()->false);
  }

  public default Trigger getAmpOverride() {
    return new Trigger(()->false);
  }

  public default Trigger getShooterOverride() {
    return new Trigger(()->false);
  }

  public default Trigger getShooterRevOverride() {
    return new Trigger(()->false);
  }

  public default Trigger getFeederOverride() {
    return new Trigger(()->false);
  }

  public default Trigger getFeederRevOverride() {
    return new Trigger(()->false);
  }
  
  public default Trigger getIntakeOverride() {
    return new Trigger(()->false);
  }
  
  public default Trigger getIntakeRevOverride() {
    return new Trigger(()->false);
  }
}
