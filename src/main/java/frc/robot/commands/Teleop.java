package frc.robot.commands;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import static frc.robot.Constants.SwerveConstants.DRIVE_FEEDFORWARD;

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
    SmartDashboard.putBoolean("Teleop/ImplicitModelFollowing", false);
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

    if(SmartDashboard.getBoolean("Teleop/ImplicitModelFollowing", false)) {
      implicitModelFollowing(speeds);
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


  // honestly since linear and angular don't affect each other, we can just treat them as X and Y and abuse the model
  double cur_kV_X = DRIVE_FEEDFORWARD.kv;
  double cur_kA_X = DRIVE_FEEDFORWARD.ka;
  double cur_kV_Y = cur_kV_X;
  double cur_kA_Y = cur_kA_X;


  double desired_kV_X = DRIVE_FEEDFORWARD.kv;
  double desired_kA_X = DRIVE_FEEDFORWARD.ka * 1.5;
  double desired_kV_Y = desired_kV_X;
  double desired_kA_Y = desired_kA_X;

  // ImplicitModelFollower

  LinearSystem<N2, N2, N2> current = LinearSystemId.identifyDrivetrainSystem(cur_kV_X, cur_kA_X, cur_kV_Y, cur_kA_Y);
  LinearSystem<N2, N2, N2> desired = LinearSystemId.identifyDrivetrainSystem(desired_kV_X, desired_kA_X, desired_kV_Y, desired_kA_Y);
  ImplicitModelFollower<N2, N2, N2> controller = new ImplicitModelFollower<>(current, desired);  


  // an attempt at implementing https://discord.com/channels/176186766946992128/368993897495527424/1220411851859300522
  void implicitModelFollowing(ChassisSpeeds speeds) {
    // need to make speeds robot relative also
    var curVel = swerve.getRobotRelativeSpeeds();
    var rot = swerve.getPose().getRotation();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      rot = rot.rotateBy(Rotation2d.fromDegrees(180));
    }
    var robotRelSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rot);
    var output = controller.calculate(
      MatBuilder.fill(Nat.N2(), Nat.N1(), curVel.vxMetersPerSecond, curVel.vyMetersPerSecond), 
      MatBuilder.fill(Nat.N2(), Nat.N1(), robotRelSpeed.vxMetersPerSecond, robotRelSpeed.vyMetersPerSecond));
    var adj_vx = output.get(0, 0);
    var adj_vy = output.get(1, 0);
    ChassisSpeeds adjSpeeds = new ChassisSpeeds(adj_vx, adj_vy, 0);
    adjSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(adjSpeeds, rot);
    speeds.vxMetersPerSecond = adjSpeeds.vxMetersPerSecond;
    speeds.vyMetersPerSecond = adjSpeeds.vyMetersPerSecond;
  }
}