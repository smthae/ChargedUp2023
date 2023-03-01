package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class IntakeIn extends CommandBase {
  private final Wrist wrist;
  private PieceType gamePieceType;
  private boolean auto = false;
  private int counter = 0;
  private LEDs leds;
  private boolean hasSeen = false;

  public IntakeIn(Wrist wrist, PieceType gamePiece, LEDs leds) {
    this.wrist = wrist;
    this.gamePieceType = gamePiece;
    this.leds = leds;
  }

  public IntakeIn(Wrist wrist, PieceType gamePiece, boolean auto) {
    this.wrist = wrist;
    this.gamePieceType = gamePiece;
    this.auto = auto;
  }

  @Override
  public void initialize() {
    if (this.wrist.getBeambreak()) {
      this.end(auto);
    } else {
      this.wrist.currentPiece = this.gamePieceType;
      this.wrist.intakeIn(this.gamePieceType);
      this.hasSeen = false;
      this.counter = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!this.wrist.getBeambreak()) {
      this.leds.set(Constants.LEDConstants.off);
    }

    this.wrist.intakeStop();
  }

  @Override
  public boolean isFinished() {
    this.leds.set(Constants.LEDConstants.solidRed);
    if (this.hasSeen) {
      counter++;
      if (counter > 10) {
        this.leds.set(Constants.LEDConstants.solidGreen);
        return true;
      }
      return false;
    }

    if (this.wrist.getBeambreak()) {
      this.hasSeen = true;
      return false;
    }
    return false;
  }

}
