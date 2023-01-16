package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeOut extends CommandBase {
    private final Wrist wrist;

    public IntakeOut(Wrist wrist) {
        this.wrist = wrist;
    }

    @Override
    public void initialize() {
        this.wrist.intakeOut();
    }

    @Override
    public void end(boolean interrupted) {
        this.wrist.intakeStop();
    }
}
