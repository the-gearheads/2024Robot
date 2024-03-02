package frc.robot;

import org.littletonrobotics.junction.Logger;

public class RobotState {
  enum GoalMode {
    AMP, SPEAKER
  }

  public static GoalMode goalMode = GoalMode.SPEAKER;

  public static void log() {
    Logger.recordOutput("CurrentGoal", goalMode.toString());
  }
}
