package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase {
  private final Swerve swerve;
  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private final Field2d field2d = new Field2d();
  private double previousPipelineTimestamp = 0;
  private AprilTagFieldLayout aprilTagFieldLayout;

  public PoseEstimator(Swerve swerve) {
    this.swerve = swerve;
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      aprilTagFieldLayout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
          : OriginPosition.kRedAllianceWallRightSide);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      aprilTagFieldLayout = null;
    }

    this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
        this.swerve.getYaw(), this.swerve.getPositions(), new Pose2d(), Constants.Swerve.stateStdDevs,
        Constants.Vision.visionMeasurementStdDevs);

    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    var pipelineResult = this.swerve.camera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty()
          : aprilTagFieldLayout.getTagPose(fiducialId);
      if (target.getPoseAmbiguity() <= 0.2 && fiducialId >= 0 && tagPose.isPresent()) {
        var targetPos = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPos.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(Constants.Vision.cameraToRobot);
        this.swerveDrivePoseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
      }
    }

    this.swerveDrivePoseEstimator.update(this.swerve.getYaw(), this.swerve.getPositions());

    field2d.setRobotPose(getCurrentPose());
  }

  public Pose2d getCurrentPose() {
    return this.swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    this.swerveDrivePoseEstimator.resetPosition(
        this.swerve.getYaw(),
        this.swerve.getPositions(),
        newPose);
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }
}
