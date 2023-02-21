package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Wrist;

public class DefaultAuto extends SequentialCommandGroup {
  public DefaultAuto(Wrist wrist) {
    addCommands(Commands.none());
  }
}
