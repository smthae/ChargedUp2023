package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class IntakeIn extends CommandBase {
  private final Wrist wrist;
  private PieceType gamePieceType;
  private boolean auto = false;
  private int counter = 0;
  // private int delay = 200;
  // private long delayCounterStart = 0;

  public IntakeIn(Wrist wrist, PieceType gamePiece) {
    this.wrist = wrist;
    this.gamePieceType = gamePiece;
  }

  public IntakeIn(Wrist wrist, PieceType gamePiece, boolean auto) {
    this.wrist = wrist;
    this.gamePieceType = gamePiece;
    this.auto = auto;
  }

  @Override
  public void initialize() {
    this.wrist.currentPiece = this.gamePieceType;
    this.wrist.intakeIn(this.gamePieceType);
    this.counter = 0;
  }

  @Override
  public void end(boolean interrupted) {
    this.wrist.intakeStop();
  }

  @Override
  public void execute() {
    counter++;
  }

  @Override
  public boolean isFinished() {
    if (this.auto) {
      return this.counter > 100;
    }

    return false;
  }

}
