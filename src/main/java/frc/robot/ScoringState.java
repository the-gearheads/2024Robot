package frc.robot;

import org.littletonrobotics.junction.Logger;

public class ScoringState {
  public enum GoalMode {
    AMP, SPEAKER, STAGE
  }

  public static GoalMode goalMode = GoalMode.SPEAKER;

  public static void log() {
    Logger.recordOutput("CurrentGoal", goalMode.toString());
  }
}
