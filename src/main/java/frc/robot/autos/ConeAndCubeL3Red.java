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
import frc.robot.commands.MoveArm;
import frc.robot.commands.presets.ConeL2;
import frc.robot.commands.presets.ConeL2Score;
import frc.robot.commands.presets.ConeL3;
import frc.robot.commands.presets.ConeL3Score;
import frc.robot.commands.presets.ConeStanding;
import frc.robot.commands.presets.CubeIntake;
import frc.robot.commands.presets.CubeL2;
import frc.robot.commands.presets.CubeL3;
import frc.robot.commands.presets.CubeL3Score;
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

public class ConeAndCubeL3Red implements AutoImpl {
        private final SwerveAutoBuilder autoBuilder;
        private final List<PathPlannerTrajectory> pathGroup;
        private final PoseEstimator poseEstimator;
        private final Swerve swerve;
        private final Arm arm;
        private final Wrist wrist;
        private final LEDs leds;

        public ConeAndCubeL3Red(Swerve swerve, PoseEstimator poseEstimator, Arm arm, Wrist wrist,
                        LEDs leds) {
                this.poseEstimator = poseEstimator;
                this.swerve = swerve;
                this.arm = arm;
                this.wrist = wrist;
                this.leds = leds;

                pathGroup = PathPlanner.loadPathGroup("cone & cube_red",
                                new PathConstraints(4,
                                                3),
                                new PathConstraints(0.5,
                                                0.5),
                                new PathConstraints(4,
                                                3));

                HashMap<String, Command> eventMap = new HashMap<>();
                eventMap.put("cubeintake", new CubeIntake(arm, wrist, leds));
                eventMap.put("intake", new IntakeIn(arm, wrist, PieceType.CUBE, leds));
                eventMap.put("cubel3", new CubeL3(arm, wrist, leds));

                autoBuilder = new SwerveAutoBuilder(poseEstimator::currentPose,
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

        public Pose2d getInitialHolonomicPose() {
                return pathGroup.get(0).getInitialHolonomicPose();
        }

        public Command getCommand() {
                return new SequentialCommandGroup(
                                new ConeL3Score(arm, wrist, leds),
                                autoBuilder.fullAuto(pathGroup),
                                new CubeL3Score(arm, wrist, leds));
        }
}
