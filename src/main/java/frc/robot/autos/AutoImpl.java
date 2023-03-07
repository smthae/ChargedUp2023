package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public interface AutoImpl {
  Pose2d getInitialHolonomicPose();

  Command getCommand();
}
