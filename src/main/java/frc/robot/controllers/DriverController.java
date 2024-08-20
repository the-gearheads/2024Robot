package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverController {

  XboxController controller;

  public DriverController(int id) {
    if(id == -1) {
      this.controller = null;
      return;
    }
    this.controller = new XboxController(id);
  }

  private boolean isNull() {
    return controller == null;
  }

  private Trigger emptyTrigger() {
    return new Trigger(() -> false);
  }

  public double getTranslateXAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getLeftY());
  }

  public double getTranslateYAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getLeftX());
  }

  public double getRotateAxis() {
    if(isNull()) return 0;
    // setRumble(Math.abs(controller.getRightX()));
    return Controllers.deadband(-controller.getRightX());
  }

  public double getSpeedUpAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(controller.getLeftTriggerAxis());
  }
  
  public double getSlowDownAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(controller.getRightTriggerAxis());
  }

  public Trigger getAlignBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRightBumper());
  }

  // only work during stage mode
  public Trigger getLClimbUpBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> Controllers.deadband(controller.getLeftTriggerAxis()) > 0); 
  }

  public Trigger getLClimbDownBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getLeftBumper());
  }

  public Trigger getRClimbUpBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> Controllers.deadband(controller.getRightTriggerAxis()) > 0); 
  }

  public Trigger getRClimbDownBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRightBumper());
  }
  // ----

  public Trigger getOverheadFeedBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getLeftBumper());
  }

  public Trigger getScoreBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> Controllers.deadband(controller.getLeftTriggerAxis()) > 0);
  }

  public Trigger getIntakeBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> Controllers.deadband(controller.getRightTriggerAxis()) > 0);
  }

  public Trigger getOuttakeBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRightBumper());
  }
  
  public Trigger getUnderstageFeedBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRawButton(17)); // left paddle
  }

  public Trigger getAmpModeBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRawButton(16));
  }

  public Trigger getBabyBirdBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getYButton());
  }

  public Trigger getStageModeBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getLeftStickButton());
  }

  public Trigger getClimbersAutoExtendBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRightStickButton());
  }

  // -- backup btns --
  public Trigger getEnableVisionBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getStartButton());
  }

  public Trigger getDisableVisionBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getBackButton());
  }

  // dpad is oh shit btns, down is reset pose, left is reconfigure. right is climber override
  public Trigger getResetPoseBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 180);
  }

  public Trigger reconfigureEverything() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 270);
  }

  public Trigger allowClimberOverride() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 90);
  }

  public void setRumble(double rumble) {
    if(isNull()) return;
    controller.setRumble(RumbleType.kBothRumble, rumble);
  }

  public Command getRumbleCommand(double rumble, double seconds) {
    return Commands.runEnd(()->setRumble(rumble), ()->setRumble(0)).withTimeout(seconds);
  }

  public Command getRumbleCommand(double rumble, double seconds, int pulses) {
    final int pulseCounter[] = {0};
    return Commands.runEnd(()->setRumble(rumble), ()->{setRumble(0);pulseCounter[0]++;}).withTimeout(seconds).andThen(new WaitCommand(seconds*1.2)).repeatedly().until(() -> pulseCounter[0]==pulses);
  }
}
