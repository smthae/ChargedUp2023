package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class ConeStanding extends ParallelCommandGroup {
    public ConeStanding(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new MoveArm(arm, -10.372419, leds),
                new MoveWrist(wrist, -0.221668, leds));
    }
}
