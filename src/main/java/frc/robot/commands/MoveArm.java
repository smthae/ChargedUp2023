package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
    private Arm arm;
    private double angle;

    public MoveArm(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        this.arm.setArmSetpoint(this.angle);
    }

    @Override
    public boolean isFinished() {
        return this.arm.atSetpoint();
    }
}
