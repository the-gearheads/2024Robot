package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

public class ScoringState {
  public enum GoalMode {
    AMP, SPEAKER, STAGE
  }

  public static GoalMode goalMode = GoalMode.SPEAKER;

  private static double babyBirdWatchdogTimerStart = Timer.getFPGATimestamp();

  public static void periodic() {
    Logger.recordOutput("CurrentGoal", goalMode.toString());
    Logger.recordOutput("BabyBirdMode", babyBirdMode);
    
    if (babyBirdMode && Timer.getFPGATimestamp() - babyBirdWatchdogTimerStart > 1) {
      babyBirdMode = false; // It's pretty bad if this gets stuck so I want to be doubly sure that it's only ever set when it needs to be
    }
  }

  public static boolean babyBirdMode = false;

  public static void feedBabyBirdMode() {
    babyBirdWatchdogTimerStart = Timer.getFPGATimestamp();
  }

}
