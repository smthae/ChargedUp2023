package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private final Spark blinkin = new Spark(Constants.blinkinPort);

  public LEDs() {
    this.forestPattern();
  }

  public void set(double value) {
    if (value >= -1.0 && value <= 1.0) {
      this.blinkin.set(value);
    }
  }

  public CommandBase forestPattern() {
    return runOnce(() -> set(-0.37));
  }

  public CommandBase solidYellow() {
    return runOnce(() -> set(0.69));
  }

  public CommandBase solidViolet() {
    return runOnce(() -> set(0.91));
  }

  public CommandBase solidGreen() {
    return runOnce(() -> set(0.77));
  }

  public CommandBase solidRed() {
    return runOnce(() -> set(0.61));
  }

}