package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;

public class AutonAutoArmHeight extends Command {
  Arm arm;
  NetworkTable table;
  DoubleArraySubscriber areasSub;
  public AutonAutoArmHeight(Arm arm) {
    this.arm = arm;
    table = NetworkTableInstance.getDefault().getTable("PathPlanner"); //TODO: move string val into constant
    areasSub = table.getDoubleArrayTopic("activePath").subscribe(new double[] {}); //TODO: move activePath into constant

  }

  @Override
  public void execute() {
    double[] path = areasSub.get();
    double endpointX = path[path.length - 3]; //TODO: move the 3 and 2 into constants
    double endpointY = path[path.length - 2];
    double armAngle = ShooterCalculations.getShooterAngle(new Translation2d(endpointX, endpointY));
    arm.setAngle(armAngle);
  }

}
