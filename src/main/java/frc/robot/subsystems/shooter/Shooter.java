package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ShooterNTControl;
import frc.robot.subsystems.FlywheelMotor;

import static frc.robot.Constants.ShooterConstants.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {

  protected FlywheelMotor topMotor = new FlywheelMotor("Shooter/Top", TOP_ID, PID, FEEDFORWARD);
  protected FlywheelMotor bottomMotor = new FlywheelMotor("Shooter/Bottom", BOTTOM_ID, PID, FEEDFORWARD);

  public Shooter() {
    setDefaultCommand(new ShooterNTControl(this));
  }

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

  Debouncer speedDebouncer = new Debouncer(0.1);
  public boolean atSpeed() {
    boolean speedWithinTolerance = Math.abs(topMotor.getVelocity() - topMotor.getVelocitySetpoint()) < SPEED_TOLERANCE &&
                             Math.abs(bottomMotor.getVelocity() - bottomMotor.getVelocitySetpoint()) < SPEED_TOLERANCE;
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
