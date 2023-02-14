package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.Wrist;

public class DefaultAuto extends SequentialCommandGroup {
  public DefaultAuto(Wrist wrist) {
    // addCommands(new IntakeIn(wrist, true), new IntakeOut(wrist, true));
    addCommands(new InstantCommand(() -> {
      System.out.println("No auto selected, running the default auto");
    }));
  }
}
