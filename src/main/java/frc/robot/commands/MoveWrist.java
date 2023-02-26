package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class MoveWrist extends CommandBase {
    private Wrist wrist;
    private double angle;
    public int done = 0;

    public MoveWrist(Wrist wrist, double angle) {
        this.wrist = wrist;
        this.angle = angle;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        this.wrist.setWristSetpoint(this.angle);
    }

    @Override
    public boolean isFinished() {
        return this.wrist.atSetpoint();
    }
}
