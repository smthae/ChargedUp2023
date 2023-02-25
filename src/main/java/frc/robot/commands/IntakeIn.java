package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class IntakeIn extends CommandBase {
  private final Wrist wrist;
  private PieceType gamePieceType;
  // private int delay = 200;
  // private long delayCounterStart = 0;

  public IntakeIn(Wrist wrist, PieceType gamePiece) {
    this.wrist = wrist;
    this.gamePieceType = gamePiece;

  }

  @Override
  public void initialize() {
    this.wrist.currentPiece = this.gamePieceType;
    this.wrist.intakeIn(this.gamePieceType);
  }

  @Override
  public void end(boolean interrupted) {
    this.wrist.intakeStop();
  }

  @Override
  public boolean isFinished() {
    if (!this.wrist.colorSensor.isConnected()) {
      return false;
    }
    PieceType gamePieceType = this.wrist.getGamePieceType();
    if (gamePieceType == PieceType.CUBE) {
      // if (this.delayCounterStart != 0 && (System.currentTimeMillis() -
      // this.delayCounterStart >= this.delay)) {
      // return true;
      // } else {
      // this.delayCounterStart = System.currentTimeMillis();
      // }
      return true;
    } else if (gamePieceType == PieceType.CONE) {
      return true;
    }
    return false;
  }

}
