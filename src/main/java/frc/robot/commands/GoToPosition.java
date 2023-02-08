package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;

public class GoToPosition extends CommandBase {
  private final Transform3d position;
  private final Swerve swerve;
  private final PoseEstimator poseEstimator;
  private final ProfiledPIDController xController = Constants.Vision.translationController;
  private final ProfiledPIDController yController = Constants.Vision.translationController;
  private final ProfiledPIDController rotationController = Constants.Vision.rotationController;
  private boolean debug = false;

  public GoToPosition(
      Swerve swerve,
      PoseEstimator poseEstimator,
      Transform3d position) {
    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    this.position = position;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    rotationController.setTolerance(Units.degreesToRadians(3));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  public void setDebug(boolean debug) {
    this.debug = debug;
  }

  @Override
  public void initialize() {
    var robotPose = this.poseEstimator.getCurrentPose();
    rotationController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose2d = this.poseEstimator.getCurrentPose();
    var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    var xSpeed = xController.calculate(this.position.getX());
    // if (xController.atGoal()) {
    // xSpeed = 0;
    // }

    var ySpeed = yController.calculate(this.position.getY());
    // if (yController.atGoal()) {
    // ySpeed = 0;
    // }

    var omegaSpeed = rotationController.calculate(this.position.getRotation().getY());
    // if (rotationController.atGoal()) {
    // omegaSpeed = 0;
    // }

    if (debug) {
      SmartDashboard.putNumber("X target", xSpeed);
      SmartDashboard.putNumber("Y target", ySpeed);
      SmartDashboard.putNumber("Omega target", omegaSpeed);
    }

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed,
            omegaSpeed,
            robotPose2d.getRotation()));
    this.swerve.setModuleStates(swerveModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.brake();
  }

  @Override
  public boolean isFinished() {
    return this.xController.atSetpoint() && this.yController.atSetpoint() && this.rotationController.atSetpoint();
  }

}