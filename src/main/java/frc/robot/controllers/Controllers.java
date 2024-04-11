package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Controllers {

  // move after variables
  private Controllers() {}

  private static String[] lastControllerNames = new String[6];

  public static DriverController driverController;
  public static OperatorController operatorController;

  /** Returns true if the connected controllers have changed since last called. */
  public static boolean didControllersChange() {
    boolean hasChanged = false;// TODO: update name to be more descriptive - what has changed?

    for (int i = 0; i < DriverStation.kJoystickPorts; i++) { //Moce the DriverStation.k into its own variable so it isn't executed on every check at the end of loop
      String name = DriverStation.getJoystickName(i);
      if (!name.equals(lastControllerNames[i])) {
        hasChanged = true;
        lastControllerNames[i] = name;
        //TODO add a break here OR, even better, replace this whole loop with a while () OR add the hasChanged var in the condition 
      }
    }

    return hasChanged;
  }

  public static void updateActiveControllerInstance() {
    // Defaults, since a NullPointerException would be far worse than any warnings
    // driverController = new DriverController() {};
    driverController = new DriverController(-1); // TODO: move the controller definitions in here if not being called anywhere else - no need to scope to whole class when just used in this method
    operatorController = new OperatorController() {};

    boolean foundDriveController = false;
    boolean foundOperatorController = false;

    for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
      String joyName = DriverStation.getJoystickName(i);
      if (joyName.equals("")) // TODO; instead of using a continue; here - create a condition that wraps the whole code in here to not execute when joyName is ""
        continue;


      if (!foundOperatorController && (joyName.contains("T.16000M") || joyName.contains("Keyboard 1"))) { //TODO: move strings into const
        foundOperatorController = true;
        operatorController = new Thrustmaster(i);
      }// TODO: remove whitespace from here to the else if

      // No filtering for now, just use the first
      else if (!foundDriveController) {
        foundDriveController = true;
        driverController = new DriverController(i);
      }
    }
  }

  public static double deadband(double num) {
    return MathUtil.applyDeadband(num, Constants.Controllers.JOYSTICK_DEADBAND);
  }
}
