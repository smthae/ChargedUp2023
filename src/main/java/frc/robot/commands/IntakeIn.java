package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class IntakeIn extends CommandBase {
  private final Wrist wrist;
  private Swerve swerve;
  private boolean auto = false;
  private int delay = 200;
  private long delayCounterStart = 0;

  public IntakeIn(Wrist wrist, Swerve swerve) {
    this.wrist = wrist;
    this.swerve = swerve;
  }

  public IntakeIn(Wrist wrist, boolean auto) {
    this.wrist = wrist;
    this.auto = auto;
  }

  @Override
  public void initialize() {
    this.wrist.intakeIn(this.wrist.currentPiece);
    if (!this.auto) {
      this.swerve.setCoR(new Translation2d(1, 0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.wrist.intakeStop();
    if (!this.auto) {
      this.swerve.resetCoR();
    }
  }

  @Override
  public void execute() {
    if (this.auto)
      return;

    PieceType gamePiece = this.wrist.getGamPieceType();
    if (gamePiece != PieceType.AIR) {
      this.wrist.intakeStop();
    } else {
      this.wrist.intakeIn(gamePiece);
    }
  }

  @Override
  public boolean isFinished() {
    if (this.auto) {
      PieceType gamePieceType = this.wrist.getGamPieceType();
      if (gamePieceType == PieceType.CUBE) {
        if (this.delayCounterStart != 0 && (System.currentTimeMillis() - this.delayCounterStart >= this.delay)) {
          return true;
        } else {
          this.delayCounterStart = System.currentTimeMillis();
        }
      } else if (gamePieceType == PieceType.CONE) {
        return true;
      }
    }
    return false;
  }

}
