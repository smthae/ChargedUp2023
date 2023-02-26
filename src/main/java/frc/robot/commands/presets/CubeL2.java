package frc.robot.commands.presets;

import javax.accessibility.AccessibleExtendedComponent;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class CubeL2 extends ParallelCommandGroup {
    public CubeL2(Arm arm, Wrist wrist) {
        addCommands(
                new MoveArm(arm, -9.355),
                new MoveWrist(wrist, 1.81986));
    }
}