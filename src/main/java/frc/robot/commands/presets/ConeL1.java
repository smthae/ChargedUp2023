package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class ConeL1 extends ParallelCommandGroup {
    public ConeL1(Arm arm, Wrist wrist) {
        addCommands(new MoveArm(arm, -10.372419), new MoveWrist(wrist, -0.221668));
    }
}