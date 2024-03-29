package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.FlywheelMotor;

import static frc.robot.Constants.ShooterConstants.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;


public class Shooter extends SubsystemBase {

  public FlywheelMotor topMotor = new FlywheelMotor("Shooter/Top", TOP_ID, PID, FEEDFORWARD, false, false);
  public FlywheelMotor bottomMotor = new FlywheelMotor("Shooter/Bottom", BOTTOM_ID, PID, FEEDFORWARD, false, false);

  public Shooter() {}

  @Override
  public void periodic() {
    topMotor.periodic();
    bottomMotor.periodic();
    topMotor.log();
    bottomMotor.log();
    Logger.recordOutput("Shooter/AtSpeed", atSpeed());
  }

  public void setSpeed(double speed) {
    topMotor.setSpeed(speed);
    bottomMotor.setSpeed(speed);
  }

  public void setTopSpeed(double speed) {
    topMotor.setSpeed(speed);
  }
  
  public void setBottomSpeed(double speed) {
    bottomMotor.setSpeed(speed);
  }

  Debouncer speedDebouncer = new Debouncer(0.4);
  public boolean atSpeed() {
    double tolerance = SPEED_TOLERANCE;
    if(Robot.isSimulation()) {
      tolerance = 3000; // sim setpoint 4k rpm turns out to be like 6661 rpm
    }
    boolean speedWithinTolerance = ((Math.abs(topMotor.getVelocity() - topMotor.getVelocitySetpoint()) < tolerance) || topMotor.getVelocity() > topMotor.getVelocitySetpoint()) &&
                             ((Math.abs(bottomMotor.getVelocity() - bottomMotor.getVelocitySetpoint()) < tolerance) || topMotor.getVelocity() > topMotor.getVelocitySetpoint());
    return speedDebouncer.calculate(speedWithinTolerance);
  }
  
  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, 
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
        (voltage) -> { topMotor.setVolts(voltage.in(Volts)); bottomMotor.setVolts(voltage.in(Volts)); },
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
  }

}
