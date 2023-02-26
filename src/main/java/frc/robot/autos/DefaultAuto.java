package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class DefaultAuto {
  Wrist wrist;
  Arm arm;

  public DefaultAuto(Wrist wrist, Arm arm) {
    this.wrist = wrist;
    this.arm = arm;
  }

  public SequentialCommandGroup getCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(new MoveArm(this.arm, 31),
            new MoveWrist(this.wrist, -0.649656)),
        Commands.waitSeconds(1),
        new IntakeOut(wrist), new ParallelCommandGroup(new MoveArm(arm, -50), new MoveWrist(wrist, 2.44)));
  }
}
