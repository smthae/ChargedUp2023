package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignAprilTag extends CommandBase {
  private final int tagToChase;
  private final Transform3d tagToGoal;
  private final Swerve swerve;
  private final Supplier<Pose2d> poseProvider;
  private PhotonTrackedTarget lastTarget;
  private final PhotonCamera camera;
  private final ProfiledPIDController xController = Constants.Vision.translationController;
  private final ProfiledPIDController yController = Constants.Vision.translationController;
  private final ProfiledPIDController rotationController = Constants.Vision.rotationController;

  public AlignAprilTag(
      Swerve swerve,
      PhotonCamera camera,
      Supplier<Pose2d> poseProvider, int tagToChase, Transform3d tagToGoal) {
    this.swerve = swerve;
    this.camera = camera;
    this.poseProvider = poseProvider;
    this.tagToChase = tagToChase;
    this.tagToGoal = tagToGoal;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    rotationController.setTolerance(Units.degreesToRadians(3));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    rotationController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    var photonRes = camera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == tagToChase)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;

        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(Constants.Vision.robotToCamera);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);

        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        rotationController.setGoal(goalPose.getRotation().getRadians());
      }
    }

    if (lastTarget == null) {
      // No target has been visible
      swerve.brake();
    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = rotationController.calculate(robotPose2d.getRotation().getRadians());
      if (rotationController.atGoal()) {
        omegaSpeed = 0;
      }

      SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed,
              omegaSpeed,
              robotPose2d.getRotation()));
      this.swerve.setModuleStates(swerveModuleStates);

    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.brake();
  }

}