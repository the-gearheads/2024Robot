package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Thrustmaster implements OperatorController {
  Joystick joy;
  public Thrustmaster(int id) {
    joy = new Joystick(id);
  }

  // this thing has a range of -0.87 to 0.87 and we want rpms between 0 and 5500
  public double getOverrideShooterSpeed() {
    double val = joy.getRawAxis(3);
    double range = 0.87;
    return (MathUtil.applyDeadband(val + range, 0.1) / (range * 2)) * 5500;
  }

  public Trigger getFeederOverride() {
    return new Trigger(()->joy.getRawButton(1));
  }

  public Trigger getIntakeOverride() {
    return new Trigger(()->joy.getRawButton(2));
  }
}
