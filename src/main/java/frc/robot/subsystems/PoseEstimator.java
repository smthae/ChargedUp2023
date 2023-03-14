package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.AllianceFlip;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase {
  private final Swerve swerve;
  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private final Field2d field2d = new Field2d();
  private double previousPipelineTimestamp = 0;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera camera;

  public PoseEstimator(Swerve swerve, PhotonCamera camera) {
    this.swerve = swerve;
    this.camera = camera;

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
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
    this.swerveDrivePoseEstimator.update(this.swerve.getYaw(), this.swerve.getPositions());
    SmartDashboard.putString("Estimated Pose", this.getFormattedPose());
    field2d.setRobotPose(currentPose());
    var result = this.camera.getLatestResult();
    if (!result.hasTargets())
      return;

    var id = result.getBestTarget().getFiducialId();
    if (id == -1 || id == 4 || id == 5)
      return;

    var tag = aprilTagFieldLayout.getTagPose(id);
    if (tag.isEmpty())
      return;

    if (result.getBestTarget().getPoseAmbiguity() < 0.3) {
      this.swerveDrivePoseEstimator.addVisionMeasurement(
          AllianceFlip.apply(tag.get()
              .plus(result.getBestTarget().getBestCameraToTarget().inverse().plus(Constants.Vision.cameraToRobot))
              .toPose2d()),
          result.getTimestampSeconds());
    }

  }

  public Pose2d currentPose() {
    return this.swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    this.swerveDrivePoseEstimator.resetPosition(
        this.swerve.getYaw(),
        this.swerve.getPositions(),
        newPose);
  }

  private String getFormattedPose() {
    var pose = currentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }
}