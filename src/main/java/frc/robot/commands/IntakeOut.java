package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Wrist;

public class IntakeOut extends CommandBase {
  private final Wrist wrist;
  private boolean auto = false;
  private PieceType pieceType = PieceType.AIR;

  public IntakeOut(Wrist wrist) {
    this.wrist = wrist;
  }

  public IntakeOut(Wrist wrist, boolean auto) {
    this.wrist = wrist;
    this.auto = auto;
  }

  @Override
  public void initialize() {
    this.pieceType = this.wrist.getGamePieceType();
    this.wrist.intakeOut(this.pieceType);
  }

  @Override
  public void end(boolean interrupted) {
    this.wrist.intakeStop();
  }

  @Override
  public boolean isFinished() {
    PieceType gamePieceType = this.wrist.getGamePieceType();
    if (gamePieceType == PieceType.AIR) {
      return true;
    }
    return false;
  }
}
