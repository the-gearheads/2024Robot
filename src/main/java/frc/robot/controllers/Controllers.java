package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Controllers {

  private Controllers() {}

  private static String[] lastControllerNames = new String[6];

  public static DriverController driverController;

  /** Returns true if the connected controllers have changed since last called. */
  public static boolean didControllersChange() {
    boolean hasChanged = false;

    for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
      String name = DriverStation.getJoystickName(i);
      if (!name.equals(lastControllerNames[i])) {
        hasChanged = true;
        lastControllerNames[i] = name;
      }
    }

    return hasChanged;
  }

  public static void updateActiveControllerInstance() {
    // Defaults, since a NullPointerException would be far worse than any warnings
    // driverController = new DriverController() {};
    driverController = new DriverController(-1);

    boolean foundDriveController = false;
    boolean foundOperatorController = false;

    for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
      String joyName = DriverStation.getJoystickName(i);
      if (joyName.equals(""))
        continue;


      if (!foundOperatorController && (joyName.contains("T.16000M") || joyName.contains("Keyboard 1"))) {
        foundOperatorController = true;
        // operatorController = new Thrustmaster(i);
      }

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
