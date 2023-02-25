package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Wrist;

public class IntakeOut extends CommandBase {
  private final Wrist wrist;

  public IntakeOut(Wrist wrist) {
    this.wrist = wrist;
  }

  @Override
  public void initialize() {
    // this.pieceType = this.wrist.getGamePieceType();
    this.wrist.intakeOut(this.wrist.currentPiece);
  }

  @Override
  public void end(boolean interrupted) {
    this.wrist.intakeStop();
  }

  @Override
  public boolean isFinished() {
    if (this.wrist.colorSensor.isConnected() && this.wrist.getGamePieceType() == PieceType.AIR) {
      return true;
    }
    return false;
  }
}
