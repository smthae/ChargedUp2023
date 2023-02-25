package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import org.photonvision.PhotonCamera;

public class exampleAuto implements AutoImpl {

  private final SwerveAutoBuilder autoBuilder;
  private final List<PathPlannerTrajectory> pathGroup;
  private final PhotonCamera camera;
  private final PoseEstimator poseEstimator;
  private final Swerve swerve;
  private final Wrist wrist;

  public exampleAuto(Swerve swerve, PhotonCamera camera, PoseEstimator poseEstimator, Wrist wrist) {
    this.camera = camera;
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.wrist = wrist;
    this.wrist.currentPiece = PieceType.CONE;

    pathGroup = PathPlanner.loadPathGroup("test4",
        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("intakein",
        new SequentialCommandGroup(new IntakeIn(wrist, PieceType.CONE),
            new InstantCommand(() -> wrist.setWristSetpoint(300))));

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
        true,
        swerve);
  }

  public Pose2d getInitialHolonomicPose() {
    return pathGroup.get(0).getInitialHolonomicPose();
  }

  public Command getCommand() {
    return new SequentialCommandGroup(
        autoBuilder.fullAuto(pathGroup), new IntakeOut(wrist));
  }
}
