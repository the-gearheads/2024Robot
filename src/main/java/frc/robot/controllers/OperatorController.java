package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorController {
  public default double getOverrideShooterSpeed() {
    return 0;
  }

  public default Trigger getFeederOverride() {
    return new Trigger(()->false);
  }

  public default Trigger getIntakeOverride() {
    return new Trigger(()->false);
  }
}
