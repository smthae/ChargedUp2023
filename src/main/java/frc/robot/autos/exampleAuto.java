package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve, PhotonCamera camera, PoseEstimator poseEstimator) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("test",
                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(poseEstimator::getCurrentPose,
                poseEstimator::setCurrentPose,
                Constants.Swerve.swerveKinematics,
                new PIDConstants(Constants.AutoConstants.translationPID.p,
                        Constants.AutoConstants.translationPID.i,
                        Constants.AutoConstants.translationPID.d),
                new PIDConstants(Constants.AutoConstants.rotationPID.p,
                        Constants.AutoConstants.rotationPID.i,
                        Constants.AutoConstants.rotationPID.d),
                s_Swerve::setModuleStates,
                eventMap,
                s_Swerve);

        addCommands(new SequentialCommandGroup(
                new InstantCommand(() -> camera.setLED(VisionLEDMode.kOn)),
                autoBuilder.fullAuto(pathGroup),
                new InstantCommand(() -> camera.setLED(VisionLEDMode.kOff)),
                new AlignAprilTag(s_Swerve, camera, poseEstimator::getCurrentPose, 2,
                        new Transform3d(new Translation3d(1.5, 0, 0), new Rotation3d(0, 0, Math.PI)))));
        // addCommands(new InstantCommand(() ->
        // s_Swerve.resetOdometry(pathGroup.get(0).getInitialHolonomicPose())),
        // autoBuilder.followPathWithEvents(pathGroup.get(0)),
        // new InstantCommand(() -> s_Swerve.camera.setLED(VisionLEDMode.kOn)),
        // autoBuilder.followPathWithEvents(pathGroup.get(1)),
        // new InstantCommand(() -> s_Swerve.camera.setLED(VisionLEDMode.kOff)));
    }
}
