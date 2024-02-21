package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {

  public static double getRealTimestamp() {
    return Timer.getFPGATimestamp();
  }

  public static void recordOutput(String key, String value) {
    SmartDashboard.putString(key, value);
  }

  public static void recordOutput(String key, double value) {
    SmartDashboard.putNumber(key, value);
  }

  public static void recordOutput(String key, boolean value) {
    SmartDashboard.putBoolean(key, value);
  }

  public static void recordOutput(String key, int value) {
    SmartDashboard.putNumber(key, value);
  }

  public static void recordOutput(String key, long value) {
    SmartDashboard.putNumber(key, value);
  }

  public static void recordOutput(String key, Sendable value) {
    SmartDashboard.putData(key, value);
  }
}