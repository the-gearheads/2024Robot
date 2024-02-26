package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.ARM_LENGTH;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Not exactly a subsystem but we do in fact need a periodic function */
public class MechanismViz extends SubsystemBase {

  private final double I2M = 0.0254;

  private final double FLOOR_LENGTH = 25.5 * I2M;
  private final double FLOOR_PT2_LENGTH = 4.25 * I2M;
  private final double ARM_TO_SHOOTER_DIST = 2.52 * I2M;
  private final double SHOOTER_WHEEL_FLAG_DIST = 1.0 * I2M;

  private final double FEEDER_WHEEL_FLAG_DIST = 4.0 * I2M;
  private final double INTAKE_WHEEL_FLAG_DIST = 6.0 * I2M;

  private final Color8Bit FLOOR_COLOR = new Color8Bit(255, 128, 128);
  private final Color8Bit SHOOTER_COLOR = new Color8Bit(128, 255, 128);
  private final Color8Bit ARM_COLOR = new Color8Bit(255, 255, 0);

  Mechanism2d mech = new Mechanism2d(1, 1);
  // cad guesstimates cause ascope wants these in meters
  MechanismRoot2d root = mech.getRoot("Shooter", 0.1032, 0.1379);
  MechanismLigament2d backFloorMech = root.append(new MechanismLigament2d("FloorPt2", FLOOR_PT2_LENGTH, 0));
  MechanismLigament2d frontFloorMech = backFloorMech.append(new MechanismLigament2d("Floor", FLOOR_LENGTH, 0));
  MechanismLigament2d arm12Mech = backFloorMech.append(new MechanismLigament2d("Arm", ARM_LENGTH/2.0, 45));
  MechanismLigament2d arm22Mech = arm12Mech.append(new MechanismLigament2d("Arm", ARM_LENGTH/2.0, 0));

  MechanismLigament2d topShooterBaseMech = arm22Mech.append(new MechanismLigament2d("TopShooterBase", ARM_TO_SHOOTER_DIST, 90)); // always perpendicular to arm
  MechanismLigament2d bottomShooterBaseMech = arm22Mech.append(new MechanismLigament2d("BottomShooterBase", ARM_TO_SHOOTER_DIST, -90));
  MechanismLigament2d topShooterMech = topShooterBaseMech.append(new MechanismLigament2d("TopShooter", SHOOTER_WHEEL_FLAG_DIST, 0));
  MechanismLigament2d bottomShooterMech = bottomShooterBaseMech.append(new MechanismLigament2d("BottomShooter", SHOOTER_WHEEL_FLAG_DIST, 0));

  MechanismLigament2d feederMech = arm12Mech.append(new MechanismLigament2d("Feeder", FEEDER_WHEEL_FLAG_DIST, 0));
  MechanismLigament2d intakeMech = root.append(new MechanismLigament2d("Intake", INTAKE_WHEEL_FLAG_DIST, 0));

  Supplier<Rotation2d> armAngle;
  Supplier<Double> topShooterPos;
  Supplier<Double> bottomShooterPos;
  Supplier<Double> intakePos;
  // I think I'll just ignore the existance of the handoff motor
  Supplier<Double> feederPos;

  public MechanismViz(Supplier<Rotation2d> armAngle, Supplier<Double> topShooterPos, Supplier<Double> bottomShooterPos, Supplier<Double> intakePos, Supplier<Double> feederPos) {
    arm12Mech.setColor(ARM_COLOR);
    arm22Mech.setColor(ARM_COLOR);
    frontFloorMech.setColor(FLOOR_COLOR);
    backFloorMech.setColor(FLOOR_COLOR);
    topShooterBaseMech.setColor(ARM_COLOR);
    bottomShooterBaseMech.setColor(ARM_COLOR);
    topShooterMech.setColor(SHOOTER_COLOR);
    bottomShooterMech.setColor(SHOOTER_COLOR);
    this.armAngle = armAngle;
    this.topShooterPos = topShooterPos;
    this.bottomShooterPos = bottomShooterPos;
    this.intakePos = intakePos;
    this.feederPos = feederPos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var armPos = armAngle.get();
    arm12Mech.setAngle(armPos);
    topShooterMech.setAngle(topShooterPos.get() * 360.0); // rotations -> degrees
    bottomShooterMech.setAngle(bottomShooterPos.get() * 360.0); // rotations -> degrees
    intakeMech.setAngle(intakePos.get() * 360.0); // rotations -> degrees
    feederMech.setAngle(feederPos.get() * 360.0); // rotations -> degrees
    Logger.recordOutput("Mechanism2d", mech);
  }

}
