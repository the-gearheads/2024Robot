package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveModule {
  CANSparkMax driveMotor;
  SparkMaxPIDController drivePID;
  RelativeEncoder driveEncoder;
  CANSparkMax steerMotor;
  SparkMaxPIDController steerPID;
  SparkMaxAbsoluteEncoder steerEncoder;

  Rotation2d offset;

  public SwerveModule(int driveMotorId, int turnMotorId, double offsetDegrees) {

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    steerMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

    drivePID = driveMotor.getPIDController();
    steerPID = steerMotor.getPIDController();

    steerEncoder = steerMotor.getAbsoluteEncoder(Type.kDutyCycle);
    steerEncoder.setPositionConversionFactor(kSteerPosFactor);
    steerEncoder.setVelocityConversionFactor(kSteerVelFactor);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(kDrivePosFactor);
    driveEncoder.setVelocityConversionFactor(kDriveVelFactor);

    this.offset = Rotation2d.fromDegrees(offsetDegrees);
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRadians(steerEncoder.getPosition()).minus(offset);
  }

  public void setAngle(Rotation2d angle) {
    steerPID.setReference(angle.plus(offset).getRadians(), ControlType.kPosition);
  }
}
