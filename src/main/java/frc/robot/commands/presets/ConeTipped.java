package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class ConeTipped extends ParallelCommandGroup {
    public ConeTipped(Arm arm, Wrist wrist) {
        addCommands(new MoveArm(arm, -49),
                new MoveWrist(wrist, 1.129295));
    }
}
