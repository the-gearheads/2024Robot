package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverController {
  public default double getTranslateXAxis() {
    return 0;
  }

  public default double getTranslateYAxis() {
    return 0;
  }

  public default double getRotateAxis() {
    return 0;
  }

  public default double getSpeedModifierAxis() {
    return 0;
  }

  public default double getSlowModifierAxis() {
    return 0;
  }

  public default Trigger getGyroZeroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPatthfindButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAlignToSpeakerBtn() {
    return new Trigger(() -> false);
  }
}
