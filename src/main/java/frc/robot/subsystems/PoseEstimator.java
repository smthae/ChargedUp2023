package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.AllianceFlip;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase {
  private final Swerve swerve;
  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera camera = new PhotonCamera(Constants.Vision.cameraName);

  public PoseEstimator(Swerve swerve) {
    this.swerve = swerve;
    this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
        this.swerve.getYaw(), this.swerve.getPositions(), new Pose2d(),
        Constants.Swerve.stateStdDevs,
        Constants.Vision.visionMeasurementStdDevs);

    try {
      AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
          .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      // Create pose estimator
      photonPoseEstimator = new PhotonPoseEstimator(
          fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, Constants.Vision.robotToCamera);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
   *         targets used to create
   *         the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public void setCurrentPose(Pose2d newPose) {
    this.swerveDrivePoseEstimator.resetPosition(
        this.swerve.getYaw(),
        this.swerve.getPositions(),
        newPose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public Pose2d currentPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  private String getFormattedPose() {
    var pose = currentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  @Override
  public void periodic() {
    swerveDrivePoseEstimator.update(swerve.getYaw(),
        swerve.getPositions());
    Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(swerveDrivePoseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      Pose2d finalPose = AllianceFlip.apply(camPose.estimatedPose.toPose2d());

      swerveDrivePoseEstimator.addVisionMeasurement(finalPose, camPose.timestampSeconds);
    }

    SmartDashboard.putString("Estimated Pose", this.getFormattedPose());

  }
}
