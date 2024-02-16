package frc.robot.controllers;


/* For any button you just need the value of, PLEASE just make it return a boolean. */
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

  public default boolean getGyroZeroButton() {
    return false;
  }

  public default boolean getPatthfindButton() {
    return false;
  }

  public default boolean getResetPoseButton() {
    return false;
  }

  public default boolean getAlignToSpeakerBtn() {
    return false;
  }

  public default boolean getIntake() {
    return false;
  }
}
