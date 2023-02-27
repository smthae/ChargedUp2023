package frc.robot.autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class Tuning implements AutoImpl {
  private final SwerveAutoBuilder autoBuilder;
  private final List<PathPlannerTrajectory> pathGroup;
  private final PoseEstimator poseEstimator;
  private final Swerve swerve;
  private final Wrist wrist;
  private final Arm arm;

  public Tuning(Swerve swerve, PoseEstimator poseEstimator, Wrist wrist, Arm arm) {
    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    this.wrist = wrist;
    this.arm = arm;

    pathGroup = PathPlanner.loadPathGroup("tuning",
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
        true,
        swerve);
  }

  public Pose2d getInitialHolonomicPose() {
    return pathGroup.get(0).getInitialHolonomicPose();
  }

  public Command getCommand() {
    return new SequentialCommandGroup(
        new Rest(arm, wrist),
        autoBuilder.fullAuto(pathGroup));
  }
}
