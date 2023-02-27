package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class ConeL1Score extends SequentialCommandGroup {
    public ConeL1Score(Arm arm, Wrist wrist) {
        addCommands(
                new ConeL1(arm, wrist),
                Commands.waitSeconds(1),
                new IntakeOut(wrist), new Rest(arm, wrist));
    }
}
