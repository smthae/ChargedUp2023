package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class ConeHP extends ParallelCommandGroup {
    public ConeHP(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new MoveArm(arm, -50.94579, leds),
                new MoveWrist(wrist, 2.443461, leds));
    }
}
