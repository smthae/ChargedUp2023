package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class ConeShelf extends ParallelCommandGroup {
    public ConeShelf(Arm arm, Wrist wrist) {
        addCommands(new MoveArm(arm, 56.41), new MoveWrist(wrist, -1.25));
    }
}
