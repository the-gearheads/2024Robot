package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.FlywheelMotor;

import static frc.robot.Constants.FeederConstants.*;

import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  FlywheelMotor feederMotor = new FlywheelMotor("Feeder", FEEDER_ID, PID, FEEDFORWARD);
  FlywheelMotor handoffMotor = new FlywheelMotor("Handoff", HANDOFF_ID, PID, FEEDFORWARD);
  public Feeder() {
    SmartDashboard.putNumber("Feeder/RunSpeed", SPEED);
    // feeder
    handoffMotor.flex.follow(feederMotor.flex);
  }

  public void periodic() {
    feederMotor.periodic();
    feederMotor.log();
  }
  
  public void run() {
    feederMotor.setSpeed(SmartDashboard.getNumber("Feeder/RunSpeed", SPEED));
  }

  public void stop() {
    feederMotor.setSpeed(0);
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, 
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
        feederMotor::setVolts,
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
  }

}
