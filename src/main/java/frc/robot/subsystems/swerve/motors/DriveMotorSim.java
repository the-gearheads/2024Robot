package frc.robot.subsystems.swerve.motors;

import static frc.robot.Constants.SwerveConstants.DRIVE_FEEDFORWARD;
import static frc.robot.Constants.SwerveConstants.DRIVE_PID;
import static frc.robot.Constants.SwerveConstants.DRIVE_POS_FACTOR;
import static frc.robot.Constants.SwerveConstants.DRIVE_VEL_FACTOR;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import java.util.OptionalDouble;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class DriveMotorSim extends DriveMotor {

  PIDController softPid = new PIDController(DRIVE_PID[0], DRIVE_PID[1], DRIVE_PID[2]);

  SwerveModuleSimulation sim;

  public DriveMotorSim(int id, int index, String modulePath, SwerveModuleSimulation sim) {
    super(id, index, modulePath);
    this.sim = sim;
  }

  // i think we need to override periodic, setVoltage, getPosition, resetEncoder, and getVelocity

  @Override
  public void periodic() {
    super.periodic();
    setVoltage(softPid.calculate(getVelocity(), driveSetpoint) + DRIVE_FEEDFORWARD.calculate(driveSetpoint));
  }

  @Override
  public void setVoltage(double volts) {
    sim.requestDriveVoltageOut(volts);
  }
  
  @Override
  public double getVoltage() {
    return sim.getDriveMotorAppliedVolts();
  }

  @Override
  public double getPosition() {
    return Units.radiansToRotations(sim.getDriveEncoderUnGearedPositionRad()) * DRIVE_POS_FACTOR;
  }

  @Override
  public OptionalDouble getPositionOptional() {
    return OptionalDouble.of(getPosition());
  }

  @Override
  public void resetEncoder() {
    return; // no-op for now ig
  }

  @Override
  public double getVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(sim.getDriveEncoderUnGearedSpeedRadPerSec()) * DRIVE_VEL_FACTOR;
  }
}
