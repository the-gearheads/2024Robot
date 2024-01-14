package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ShooterNTControl;

import static frc.robot.Constants.ShooterConstants.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  protected ShooterMotor topMotor = new ShooterMotor(TOP_ID);
  protected ShooterMotor bottomMotor = new ShooterMotor(BOTTOM_ID);

  public Shooter() {
    setDefaultCommand(new ShooterNTControl(this));
  }

  @Override
  public void periodic() {
    topMotor.log("Top");
    bottomMotor.log("Bottom");
    Logger.recordOutput("Shooter/AtSpeed", atSpeed());
  }

  public void setSpeed(double speed) {
    topMotor.setSpeed(speed);
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
