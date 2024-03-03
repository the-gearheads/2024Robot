package frc.robot.subsystems;

import static frc.robot.Constants.FeederConstants.BEAMBREAK_SWITCH_ID;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class NoteSimMgr extends SubsystemBase {
  Supplier<Pose2d> robotPoseSupplier;
  Supplier<Double> shooterTopSpeedSupplier;
  Supplier<Double> shooterBottomSpeedSupplier;
  Supplier<Double> intakeSpeedSupplier;
  Supplier<Double> feederSpeedSupplier;

  DIOSim noteSwitchSim = new DIOSim(BEAMBREAK_SWITCH_ID);

  enum NoteState {
    EMPTY, INTOOK
  }

  NoteState noteState = NoteState.EMPTY;


  public NoteSimMgr(Supplier<Pose2d> robotPoseSupplier, Supplier<Double> shooterTopSpeedSupplier, Supplier<Double> shooterBottomSpeedSupplier, Supplier<Double> intakeSpeedSupplier, Supplier<Double> feederSpeedSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.shooterTopSpeedSupplier = shooterTopSpeedSupplier;
    this.shooterBottomSpeedSupplier = shooterBottomSpeedSupplier;
    this.intakeSpeedSupplier = intakeSpeedSupplier;
    this.feederSpeedSupplier = feederSpeedSupplier;
    noteSwitchSim.setIsInput(true);
  }

  Debouncer regularShootingDebouncer = new Debouncer(0.5);
  Debouncer ampShootingDebouncer = new Debouncer(0.5);

  @Override
  public void periodic() {
    if(Robot.isReal()) return;
    double tShoot = shooterTopSpeedSupplier.get();
    double bShoot = shooterBottomSpeedSupplier.get();
    double intake = intakeSpeedSupplier.get();
    double feeder = feederSpeedSupplier.get();
    Translation2d robotPose = robotPoseSupplier.get().getTranslation();

    if(isRunning(intake) && isRunning(feeder) && isInNote(robotPose)) {
      noteState = NoteState.INTOOK;
    }

    if(regularShootingDebouncer.calculate(isRunning(feeder) && isRunningForwards(tShoot) && isRunningForwards(bShoot) && noteState == NoteState.INTOOK)) {
      // Regular note shot
      noteState = NoteState.EMPTY;
    }

    if(ampShootingDebouncer.calculate(isRunning(feeder) && isRunningBackwards(tShoot) && isRunningForwards(bShoot) && noteState == NoteState.INTOOK)) {
      // Amp shot
      noteState = NoteState.EMPTY;
    }

    var noteSwitchState = !(noteState == NoteState.INTOOK);
    Logger.recordOutput("NoteSim/noteState", noteState.toString());
    Logger.recordOutput("NoteSim/noteSwitchSim", noteSwitchState);
    noteSwitchSim.setValue(noteSwitchState); // normally closed
  }

  public boolean isNoteInRobot() {
    return noteState == NoteState.INTOOK;
  }

  // totally enough decimal places (also these are probably a couple inches off but whatever)
  private final Translation2d[] NOTE_POSITIONS = {
    new Translation2d(2.893739700317383, 4.12746000289917), // BF1
    new Translation2d(2.8969109058380127, 5.57454776763916), // BF2
    new Translation2d(2.8846659660339355, 7.040114879608154), // BF3

    new Translation2d(13.667075157165527, 4.116274356842041), // RF1
    new Translation2d(13.651276588439941, 5.577609062194824), // RF2
    new Translation2d(13.651276588439941, 7.0310444831848145), // RF3

    new Translation2d(8.286864280700684, 0.7634815573692322), // C1
    new Translation2d(8.286864280700684, 2.4456608295440674), // C2
    new Translation2d(8.279884338378906, 4.1320719718933105), // C3
    new Translation2d(8.279884338378906, 5.806682586669922), // C4
    new Translation2d(8.279884338378906, 7.473394393920898), // C5
  };

  private final double POS_TOLERANCE = Units.inchesToMeters(4);

  boolean isInNote(Translation2d robotPos) {
    for (Translation2d notePos : NOTE_POSITIONS) {
      if (robotPos.getDistance(notePos) < POS_TOLERANCE) {
        return true;
      }
    }
    return false;
  }

  private final double RUNNING_RPM_THRESOLD = 1000;
  private boolean isRunning(double speed) {
    return Math.abs(speed) > RUNNING_RPM_THRESOLD;
  }

  private boolean isRunningForwards(double speed) {
    return speed > RUNNING_RPM_THRESOLD;
  }

  private boolean isRunningBackwards(double speed) {
    return speed < -RUNNING_RPM_THRESOLD;
  }
}
