package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/* pretty basic and naive but it does mostly work */
public class SwerveModuleSim extends SwerveModule {

  double integratedPosition = 0;
  double lastTime = Timer.getFPGATimestamp();

  double actualTargetSpeed = 0;
  double actualTargetAngle = 0;

  public SwerveModuleSim(int id, String moduleName) {
    super(id, moduleName);
  }

  @Override
  public void periodic() {
    super.periodic();
    double now = Timer.getFPGATimestamp();
    integratedPosition +=  actualTargetSpeed * (now - lastTime);
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void resetEncoders() {
    integratedPosition = 0;
  }

  @Override
  public double getPosition() {
    return integratedPosition;
  }

  @Override
  public double getDriveVelocity() {
    return actualTargetSpeed;
  }

  @Override
  public double getSteerVelocity() {
    return 0;
  }

  @Override
  protected Rotation2d getAngle() {
    return Rotation2d.fromRadians(actualTargetAngle);
  }

  @Override
  public void setState(SwerveModuleState state) {
    /* not sure if the issue is with optimization or the offsetting is messing with things but
     * we just can bypass both by doing this
     */
    actualTargetSpeed = state.speedMetersPerSecond;
    actualTargetAngle = state.angle.getRadians();
    super.setState(state);
  }
}
