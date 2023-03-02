package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class CubeL3 extends ParallelCommandGroup {
    public CubeL3(Arm arm, Wrist wrist, LEDs leds) {
        // 39.44
        // 1.509
        addCommands(
                new MoveArm(arm, 17.462, leds),
                new MoveWrist(wrist, 1.584508, leds));
    }
}