package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FeederConstants.*;

public class Intake extends SubsystemBase {
  PIDController intakePID = new PIDController(FEEDER_PID[0], FEEDER_PID[1], FEEDER_PID[2]);
  // SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(FEEDER_FF[0], FEEDER_FF[1], FEEDER_FF[2]);
  public Intake() {}

  public void periodic() {

  }
  
  public void setSetpoint(double speed) {
    intakePID.setSetpoint(speed);
  }


}
