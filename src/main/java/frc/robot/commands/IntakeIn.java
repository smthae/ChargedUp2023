package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Wrist;

public class IntakeIn extends CommandBase {
  private final Wrist wrist;
  private boolean auto = false;

  public IntakeIn(Wrist wrist) {
    this.wrist = wrist;
  }

  public IntakeIn(Wrist wrist, boolean auto) {
    this.wrist = wrist;
    this.auto = auto;
  }

  @Override
  public void initialize() {
    this.wrist.intakeIn();
  }

  @Override
  public void end(boolean interrupted) {
    this.wrist.intakeStop();
  }

  @Override
  public void execute() {
    if (this.auto)
      return;

    PieceType gamePiece = this.wrist.getGamPieceType();
    if (gamePiece != PieceType.AIR) {
      this.wrist.intakeStop();
    } else {
      this.wrist.intakeIn();
    }
  }

  @Override
  public boolean isFinished() {
    if (this.auto) {
      PieceType gamePieceType = this.wrist.getGamPieceType();
      if (gamePieceType != PieceType.AIR) {
        return true;
      }
    }
    return false;
  }

}
