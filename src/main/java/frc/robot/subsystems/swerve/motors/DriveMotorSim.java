package frc.robot.subsystems.swerve.motors;

import static frc.robot.Constants.SwerveConstants.DRIVE_FEEDFORWARD;
import static frc.robot.Constants.SwerveConstants.DRIVE_RATIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class DriveMotorSim extends DriveMotor {

  FlywheelSim sim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(DRIVE_FEEDFORWARD.kv, DRIVE_FEEDFORWARD.ka), DCMotor.getNeoVortex(1), DRIVE_RATIO);

  double pos = 0;

  public DriveMotorSim(int id, int index, String modulePath) {
    super(id, index, modulePath);
  }

  // i think we need to override periodic, setVoltage, getPosition, resetEncoder, and getVelocity

  @Override
  public void periodic() {
    super.periodic();
    sim.update(0.02);
    pos += sim.getAngularVelocityRadPerSec() * 0.02;
  }

  @Override
  public void setVoltage(double volts) {
    double batteryVolts = RobotController.getBatteryVoltage();
    sim.setInputVoltage(MathUtil.clamp(volts, -batteryVolts, batteryVolts));
  } 

  @Override
  public double getPosition() {
    return pos;
  }

  @Override
  public void resetEncoder() {
    pos = 0;
  }

  @Override
  public double getVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }
}
