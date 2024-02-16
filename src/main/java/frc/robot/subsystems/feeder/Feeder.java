package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.FeederConstants.*;

import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  // FlywheelMotor motor = new FlywheelMotor("Feeder", ID, PID, FEEDFORWARD);
  public Feeder() {
    SmartDashboard.putNumber("Feeder/RunSpeed", SPEED);
  }

  public void periodic() {
    // motor.periodic();
    // motor.log();
  }
  
  public void run() {
    // motor.setSpeed(SmartDashboard.getNumber("Intake/RunSpeed", SPEED));
  }

  public void stop() {
    // motor.setSpeed(0);
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, 
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
        // motor::setVolts,
        null,
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
  }

}
