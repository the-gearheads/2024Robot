package frc.robot.subsystems.swerve.motors;

import static frc.robot.Constants.SwerveConstants.STEER_PIDF;

import java.util.OptionalDouble;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class SteerMotorSim extends SteerMotor {
  SwerveModuleSimulation sim;
  public SteerMotorSim(int id, int index, Rotation2d offset, String modulePath, SwerveModuleSimulation sim) {
    super(id, index, offset, modulePath);
    this.sim = sim;
  }

  PIDController softPid = new PIDController(STEER_PIDF[0], STEER_PIDF[1], STEER_PIDF[2]);

  // override noOffsetGetAngle, periodic, setVoltage

  @Override
  public Rotation2d getAngle() {
    return sim.getSteerAbsoluteFacing();
  }

  @Override
  public double getAngleRadians() {
    return sim.getSteerAbsoluteFacing().getRadians();
  }

  @Override
  public OptionalDouble getAngleRadiansOptional() {
    return OptionalDouble.of(getAngleRadians());
  }

  @Override
  public void periodic() {
    var volts = softPid.calculate(getAngleRadians(), targetAngle);
    setVoltage(volts);
  }

  @Override
  public void setAngle(Rotation2d angle) {
    super.setAngle(angle);
  }

  @Override
  public void setVoltage(double volts) {
    sim.requestSteerVoltageOut(volts);
  }
}
