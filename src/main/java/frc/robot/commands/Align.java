package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.AMP_SCORE_POSE;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.swerve.Swerve;

public class Align extends Command {
  Swerve swerve;
  Arm arm;

  public Align(Swerve swerve, Arm arm) {
    this.arm = arm;
    this.swerve = swerve;
    addRequirements(swerve, arm);
  }

  @Override
  public void execute() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        swerve.getPose(),
        AMP_SCORE_POSE
    );

    PathPlannerPath ampPath = new PathPlannerPath(
        bezierPoints,
        AutoConstants.PATHFIND_CONSTRAINTS,
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
    );
    
  }
}
