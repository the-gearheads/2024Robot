package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final CANSparkFlex leftClimber = new CANSparkFlex(0, MotorType.kBrushless); 
  private final CANSparkFlex rightClimber = new CANSparkFlex(0, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  public Climber() {
    leftEncoder = leftClimber.getEncoder();
    rightEncoder = rightClimber.getEncoder();
  }

  public void extendArms() {
    rightClimber.set(1);
    leftClimber.set(1);
  }
}
