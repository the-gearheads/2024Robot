package frc.robot;

import org.littletonrobotics.junction.Logger;

public class ScoringState {
  public enum GoalMode {
    AMP, SPEAKER, STAGE
  }

  public static GoalMode goalMode = GoalMode.SPEAKER;

  public static void periodic() {
    Logger.recordOutput("CurrentGoal", goalMode.toString());
    Logger.recordOutput("BabyBirdMode", babyBirdMode);
    babyBirdMode = false; // It's pretty bad if this gets stuck so I want to be doubly sure that it's only ever set when it needs to be
  }

  public static boolean babyBirdMode = false;
}
