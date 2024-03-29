package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.ARM_LENGTH;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;

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

  private final Color8Bit ARM_NOTED_COLOR = new Color8Bit(244, 119, 2);

  private final double ARM_NOTE_PART_LENGTH = 0.3743237438;
  private final double NOTE_HEIGHT_OFFSET = Units.inchesToMeters(2.5);

  private final Translation3d ROBOT_ARM_TRANSLATION = new Translation3d(-0.202515, -0.1843, 0.196928);
  private final double ROBOT_ARM_OFFSET = Units.degreesToRadians(90+19.5);
  private final Translation3d NOTE_TRANSLATION = new Translation3d(0.202515, 0, 0.196928);
  /* 
    > 0.2316m + 4in
      (0.2316 meter) + (4 inches) = 333.2 mm

    > 332.2mm to in
      332.2 millimeters = approx. 13.07874016 in

    > 332.2mm to m
      332.2 millimeters = 0.3322 m

    > -0.202515 + 0.03
      (-0.202515) + 0.03 = -0.172515

    > 0.3322^2 + 0.172515^2
      0.3322^2 + 0.172515^2 = approx. 0.1401182652

    > sqrt( 0.1401182652)
      sqrt(0.1401182652) = approx. 0.3743237438
  */

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
  Supplier<Boolean> noteSwitch;

  Swerve swerve;

  public MechanismViz(Swerve swerve, Supplier<Rotation2d> armAngle, Supplier<Double> topShooterPos, Supplier<Double> bottomShooterPos, Supplier<Double> intakePos, Supplier<Double> feederPos, Supplier<Boolean> noteSwitch) {
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
    this.noteSwitch = noteSwitch;
    this.swerve = swerve;
  }

  @Override
  public void periodic() {    // This method will be called once per scheduler run
    var armPos = armAngle.get();
    arm12Mech.setAngle(armPos);
    topShooterMech.setAngle(topShooterPos.get() * 360.0); // rotations -> degrees
    bottomShooterMech.setAngle(bottomShooterPos.get() * 360.0); // rotations -> degrees
    intakeMech.setAngle(intakePos.get() * 360.0); // rotations -> degrees
    feederMech.setAngle(feederPos.get() * 360.0); // rotations -> degrees
    if(noteSwitch.get()) {
      arm12Mech.setColor(ARM_NOTED_COLOR);
    } else {
      arm12Mech.setColor(ARM_COLOR);
    }
    Logger.recordOutput("Mechanism2d", mech);


    Logger.recordOutput("ComponentPoses", new Transform3d[] {
      new Transform3d(
        ROBOT_ARM_TRANSLATION,
        new Rotation3d(0, ROBOT_ARM_OFFSET-armPos.getRadians(), 0)
      )
    });

    var robotPose2d = swerve.getPose();
    var robotPose3d = new Transform3d(
      new Translation3d(robotPose2d.getX(), robotPose2d.getY(), 0),
      new Rotation3d(0, 0, robotPose2d.getRotation().getRadians())
    );

    if(noteSwitch.get()) {
      Logger.recordOutput("RobotNote", new Transform3d(
        NOTE_TRANSLATION.rotateBy(new Rotation3d(0, -armPos.getRadians()-0.17, 0)).plus(new Translation3d(0, 0, NOTE_HEIGHT_OFFSET)).rotateBy(robotPose3d.getRotation()).plus(robotPose3d.getTranslation()),
        new Rotation3d(0, armPos.getRadians() + 0.17, Math.PI).plus(robotPose3d.getRotation())
      )); 
    } else {
      Logger.recordOutput("RobotNote", new Transform3d(new Translation3d(-100, -100, -100), new Rotation3d()));
    }
  }

}
