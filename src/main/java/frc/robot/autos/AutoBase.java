package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.Flipper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class AutoBase {
  public PoseEstimator poseEstimator;
  public Swerve swerve;
  public Arm arm;
  public Wrist wrist;
  public LEDs leds;
  public SwerveAutoBuilder autoBuilder;
  public List<PathPlannerTrajectory> pathGroup;
  public List<PathPlannerTrajectory> pathGroup_red;

  public Pose2d getInitialHolonomicPose() {
    return getPathGroup().get(0).getInitialHolonomicPose();
  }

  public List<PathPlannerTrajectory> getPathGroup() {
    return Flipper.shouldFlip() ? pathGroup_red : pathGroup;
  }

  public Command getCommand() {
    return Commands.none();
  }
}
