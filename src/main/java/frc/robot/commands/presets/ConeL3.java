package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class ConeL3 extends ParallelCommandGroup {
    public ConeL3(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new MoveArm(arm, 37, leds),
                new MoveWrist(wrist, 0.149669, leds));
    }
}
