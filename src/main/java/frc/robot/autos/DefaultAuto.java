package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DefaultAuto extends SequentialCommandGroup {
  public DefaultAuto() {
    addCommands(new InstantCommand(() -> {
      System.out.println("No auto selected, running the default auto");
    }));
  }
}
