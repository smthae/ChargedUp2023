package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeIn extends CommandBase {
    private final Wrist wrist;

    public IntakeIn(Wrist wrist) {
        this.wrist = wrist;
    }

    @Override
    public void initialize() {
        this.wrist.intakeIn();
    }

    @Override
    public void end(boolean interrupted) {
        this.wrist.intakeStop();
    }
}
