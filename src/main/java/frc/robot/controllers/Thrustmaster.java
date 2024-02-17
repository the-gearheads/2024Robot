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
    double range = 1.0;
    double maxSpeed = 6000;
    return (joy.getRawButton(11) ? -1 : 1) * MathUtil.clamp(maxSpeed-((MathUtil.applyDeadband(val + range, 0.1) / (range * 2)) * maxSpeed), 0, maxSpeed);
  }

  public Trigger getFeederOverride() {
    return new Trigger(()->joy.getRawButton(1));
  }

  public Trigger getIntakeOverride() {
    return new Trigger(()->joy.getRawButton(2));
  }

  public Trigger getIntakeRevOverride() {
    return new Trigger(()->joy.getRawButton(13));
  }

  public Trigger getFeederRevOverride() {
    return new Trigger(()->joy.getRawButton(12));
  }

  public Trigger getAmpOverride() {
    return new Trigger(()->joy.getRawButton(16));
  }
}
