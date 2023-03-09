package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryUtil.TrajectorySerializationException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.Flipper;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.presets.ConeL2;
import frc.robot.commands.presets.ConeL2Score;
import frc.robot.commands.presets.ConeL3;
import frc.robot.commands.presets.ConeL3Score;
import frc.robot.commands.presets.ConeStanding;
import frc.robot.commands.presets.CubeIntake;
import frc.robot.commands.presets.CubeL2;
import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class ConeAndCube implements AutoImpl {
  private final SwerveAutoBuilder autoBuilder;
  private final List<PathPlannerTrajectory> pathGroup;
  private final PhotonCamera camera;
  private final PoseEstimator poseEstimator;
  private final Swerve swerve;
  private final Arm arm;
  private final Wrist wrist;
  private final LEDs leds;

  public ConeAndCube(Swerve swerve, PhotonCamera camera, PoseEstimator poseEstimator, Arm arm, Wrist wrist,
      LEDs leds) {
    this.camera = camera;
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.arm = arm;
    this.wrist = wrist;
    this.leds = leds;

    pathGroup = PathPlanner.loadPathGroup("cone & cube",
        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("cubepickup", new CubeIntake(arm, wrist, leds));
    eventMap.put("intakein", new IntakeIn(arm, wrist, PieceType.CUBE, leds));
    eventMap.put("rest", new Rest(arm, wrist, leds));
    eventMap.put("cubel2", new CubeL2(arm, wrist, leds));
    eventMap.put("outake", new IntakeOut(arm, wrist, leds));
    eventMap.put("conel2score", new ConeL2Score(arm, wrist, leds));

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
        false,
        swerve);
  }

  public CommandBase getFullAuto(List<PathPlannerTrajectory> trajectories) {
    boolean shouldFlip = Flipper.shouldFlip();
    if (shouldFlip) {
      for (ListIterator<PathPlannerTrajectory> iter = trajectories.listIterator(); iter.hasNext();) {
        iter.set(Flipper.allianceFlip(iter.next()));
      }
    }

    return autoBuilder.fullAuto(trajectories);
  }

  public Pose2d getInitialHolonomicPose() {
    return pathGroup.get(0).getInitialHolonomicPose();
  }

  public Command getCommand() {
    return new SequentialCommandGroup(
        new Rest(arm, wrist, leds),
        new ConeL2(arm, wrist, leds),
        getFullAuto(pathGroup),
        new IntakeOut(arm, wrist, leds), new Rest(arm, wrist, leds));
  }
}
