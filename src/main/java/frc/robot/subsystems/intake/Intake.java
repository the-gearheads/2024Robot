package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.FlywheelMotor;

import static frc.robot.Constants.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  FlywheelMotor motor = new FlywheelMotor("Intake", ID, PID, FEEDFORWARD, false, false);
  public Intake() {
    SmartDashboard.putNumber("Intake/RunSpeed", SPEED);
  }

  public void periodic() {
    motor.periodic();
    motor.log();
  }
  
  public void run() {
    motor.setSpeed(SmartDashboard.getNumber("Intake/RunSpeed", SPEED));
  }

  public void runReverse() {
    motor.setSpeed(-SmartDashboard.getNumber("Intake/RunSpeed", SPEED));
  }

  public void stop() {
    motor.setSpeed(0);
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, 
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
        motor::setVolts,
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
  }

}
