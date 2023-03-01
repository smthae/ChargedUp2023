package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private final Spark blinkin = new Spark(Constants.LEDConstants.blinkinPort);

  public LEDs() {
    this.set(Constants.LEDConstants.forestPattern);
  }

  public void set(double value) {
    if (value >= -1.0 && value <= 1.0) {
      this.blinkin.set(value);
    }
  }
}