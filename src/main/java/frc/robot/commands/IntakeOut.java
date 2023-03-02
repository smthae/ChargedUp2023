package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class IntakeOut extends CommandBase {
  private final Wrist wrist;
  private final LEDs leds;
  private int counter = 0;

  public IntakeOut(Wrist wrist, LEDs leds) {
    this.wrist = wrist;
    this.leds = leds;
  }

  @Override
  public void initialize() {
    this.wrist.intakeOut(this.wrist.currentPiece);
    counter = 0;
  }

  @Override
  public void end(boolean interrupted) {
    this.wrist.intakeStop();
  }

  @Override
  public boolean isFinished() {
    if (!this.wrist.getBeambreak()) {
      if (counter > 10) {
        return true;
      }
      counter++;
      this.leds.set(Constants.LEDConstants.solidBlue);
      return false;
    }
    return false;
  }
}
