package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlignAprilTag;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class exampleAuto2 implements AutoImpl {
  private final SwerveAutoBuilder autoBuilder;
  private final List<PathPlannerTrajectory> pathGroup;
  private final PhotonCamera camera;
  private final PoseEstimator poseEstimator;
  private final Swerve swerve;

  public exampleAuto2(Swerve swerve, PhotonCamera camera, PoseEstimator poseEstimator) {
    this.camera = camera;
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    pathGroup = PathPlanner.loadPathGroup("test2",
        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    HashMap<String, Command> eventMap = new HashMap<>();

    autoBuilder = new SwerveAutoBuilder(poseEstimator::getCurrentPose,
        poseEstimator::setCurrentPose,
        Constants.Swerve.swerveKinematics,
        new PIDConstants(Constants.AutoConstants.translationPID.p,
            Constants.AutoConstants.translationPID.i,
            Constants.AutoConstants.translationPID.d),
        new PIDConstants(Constants.AutoConstants.rotationPID.p,
            Constants.AutoConstants.rotationPID.i,
            Constants.AutoConstants.rotationPID.d),
        swerve::setModuleStates,
        eventMap,
        swerve);
  }

  public Pose2d getInitialHolonomicPose() {
    return pathGroup.get(0).getInitialHolonomicPose();
  }

  public Command getCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> camera.setLED(VisionLEDMode.kOn)),
        autoBuilder.fullAuto(pathGroup),
        new InstantCommand(() -> camera.setLED(VisionLEDMode.kOff))
    // new AlignAprilTag(swerve, camera, poseEstimator::getCurrentPose, 2,
    // new Transform3d(new Translation3d(1.5, 0, 0),
    // new Rotation3d(0, 0, Math.PI)))
    );
  }
}
