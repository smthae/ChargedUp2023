package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PieceType;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class ConeL3Score extends SequentialCommandGroup {
    public ConeL3Score(Arm arm, Wrist wrist) {
        wrist.currentPiece = PieceType.CONE;
        addCommands(
                new Rest(arm, wrist),
                new ConeL2(arm, wrist),
                new ConeL3(arm, wrist),
                Commands.waitSeconds(0.5),
                new IntakeOut(wrist), new Rest(arm, wrist));
    }
}