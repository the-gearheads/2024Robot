package frc.robot.commands;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ImplicitModelFollower;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ScoringState;
import frc.robot.ScoringState.GoalMode;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.ShooterCalculations;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.GPDetect;
import frc.robot.util.BetterBangBang;

import static frc.robot.Constants.Controllers.*;
import org.littletonrobotics.junction.Logger;

public class Teleop extends Command {

  Swerve swerve;
  GPDetect gpDetect;

  PIDController noteTransController = new PIDController(0.1, 0, 0.25);
  
  public Teleop(Swerve swerve, GPDetect gpDetect) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.gpDetect = gpDetect;
    SmartDashboard.putData("Teleop/Headingcontroller", headingController);
    SmartDashboard.putData("Teleop/NoteController", noteTransController);
  }

  @Override
  public void initialize() {
    headingController.setSetpoint(swerve.getGyroRotation().getRadians());
    SmartDashboard.putBoolean("Teleop/HeadingPID", true);
    SmartDashboard.putBoolean("Swerve/FieldRelative", true);
  }

  @Override
  public void execute() {
    gpDetect.getNearestNote();
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    double rot = Controllers.driverController.getRotateAxis();

    var attemptingToRotate = MathUtil.applyDeadband(rot, 0.02) != 0; 

    double speedMod = Math.abs(Math.pow(Controllers.driverController.getSpeedUpAxis(), 2));
    double slowMod = Math.abs(Math.pow(Controllers.driverController.getSlowDownAxis(), 2));

    x = Math.pow(x, 2) * Math.signum(x);
    y = Math.pow(y, 2) * Math.signum(y);
    rot = Math.pow(rot, 3);

    // maybe consider throwing a slewratelimiter here
    
    x /= MathUtil.interpolate(1, MOD_ROT_SLOW_FACTOR, slowMod);
    y /= MathUtil.interpolate(1, MOD_ROT_SLOW_FACTOR, slowMod);
    rot /= MathUtil.interpolate(1, MOD_ROT_SLOW_FACTOR, slowMod);

    x *= BASE_TRANS_SPEED * (1 + speedMod * MOD_TRANS_SPEED_FACTOR);
    y *= BASE_TRANS_SPEED * (1 + speedMod * MOD_TRANS_SPEED_FACTOR);
    rot *= BASE_ROT_SPEED * (1 + speedMod * MOD_ROT_SPEED_FACTOR);

    var speeds = new ChassisSpeeds(x, y, rot);

    double calculatedForcedAngle = ShooterCalculations.getYaw(swerve.getPose().getTranslation()).getRadians();
    /* I don't like this anymore */
    boolean shouldAlign = Controllers.driverController.getAlignBtn().getAsBoolean() || 
                          Controllers.driverController.getAutoShootBtn().getAsBoolean() ||
                          Controllers.driverController.getAimAndFeedBtn().getAsBoolean() ||
                          Controllers.driverController.getFeedAlign().getAsBoolean() ||
                          ScoringState.goalMode == GoalMode.STAGE || 
                          (ScoringState.babyBirdMode && ShooterCalculations.isInSource(swerve.getPose().getTranslation()));
    // i think the first condition should be removed tbh but i dont want to break anything
    var forcedAngle = shouldAlign ? calculatedForcedAngle : null;
    if (DriverStation.isAutonomous()) forcedAngle = null;
    if(forcedAngle != null) headingController.setSetpoint(swerve.getGyroRotation().getRadians());

    if (SmartDashboard.getBoolean("Teleop/HeadingPID", true)) {
      headingPid(attemptingToRotate, speeds);
    }

    Logger.recordOutput("Swerve/Teleop/Speeds", speeds);
    
    if (SmartDashboard.getBoolean("Swerve/FieldRelative", true) && ScoringState.goalMode != GoalMode.STAGE) {
      swerve.driveFieldRelative(speeds, forcedAngle);
    } else {
      swerve.drive(speeds, forcedAngle);
    }

 }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0, 0, 0));
  }

  private double touchedRotateAt = Timer.getFPGATimestamp();
  private BetterBangBang headingController = new BetterBangBang();
  private void headingPid(boolean attemptingToRotate, ChassisSpeeds speeds) {
    headingController.setTolerance(0.06);
    touchedRotateAt = attemptingToRotate ? Timer.getFPGATimestamp() : touchedRotateAt;
    double timeSinceLastRotate = Timer.getFPGATimestamp() - touchedRotateAt;
    if (!attemptingToRotate && timeSinceLastRotate > 0.15) {
      double rot = headingController.calculate(swerve.getGyroRotation().getRadians());
      
      speeds.omegaRadiansPerSecond += rot;
    } else {
      headingController.setSetpoint(swerve.getGyroRotation().getRadians());
    }
  }


  double kVLinear = 0.1;
  double kVAngular = 0.1;
  double kALinear = 0.1;
  double kAAngular = 0.1;
  LinearSystem<N2, N2, N2> identifyDrivetrainSystem(
      double kVLinear, double kALinear, double kVAngular, double kAAngular) {
    if (kVLinear <= 0.0) {
      throw new IllegalArgumentException("Kv,linear must be greater than zero.");
    }
    if (kALinear <= 0.0) {
      throw new IllegalArgumentException("Ka,linear must be greater than zero.");
    }
    if (kVAngular <= 0.0) {
      throw new IllegalArgumentException("Kv,angular must be greater than zero.");
    }
    if (kAAngular <= 0.0) {
      throw new IllegalArgumentException("Ka,angular must be greater than zero.");
    }

    final double A1 = 0.5 * -(kVLinear / kALinear + kVAngular / kAAngular);
    final double A2 = 0.5 * -(kVLinear / kALinear - kVAngular / kAAngular);
    final double B1 = 0.5 * (1.0 / kALinear + 1.0 / kAAngular);
    final double B2 = 0.5 * (1.0 / kALinear - 1.0 / kAAngular);

    return new LinearSystem<>(
        MatBuilder.fill(Nat.N2(), Nat.N2(), A1, A2, A2, A1),
        MatBuilder.fill(Nat.N2(), Nat.N2(), B1, B2, B2, B1),
        // implicit model follower doesnt even really use these two
        MatBuilder.fill(Nat.N2(), Nat.N2(), 1, 0, 0, 1),
        MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 0, 0, 0));
  }

  // ImplicitModelFollower

  

  // an attempt at implementing https://discord.com/channels/176186766946992128/368993897495527424/1220411851859300522
  void implicitModelFollowing(ChassisSpeeds speeds) {
  }
}