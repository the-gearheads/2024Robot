package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;

public class AutonAutoArmHeight extends Command {
  Arm arm;
  NetworkTable table;
  StructArraySubscriber<Pose2d> areasSub;
  public AutonAutoArmHeight(Arm arm) {
    this.arm = arm;
    table = NetworkTableInstance.getDefault().getTable("PathPlanner");
    areasSub = table.getStructArrayTopic("activePath", Pose2d.struct).subscribe(new Pose2d[] {});

  }

  @Override
  public void execute() {
    Pose2d[] path = areasSub.get();
    Pose2d endpoint = path[path.length - 1];
    double armAngle = ShooterCalculations.getShooterAngle(new Translation2d(endpoint.getX(), endpoint.getY()));
    arm.setAngle(armAngle);
  }

}
