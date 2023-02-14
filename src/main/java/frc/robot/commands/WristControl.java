package frc.robot.commands;

import java.lang.reflect.Executable;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class WristControl extends CommandBase {
  private Wrist wrist;
  private DoubleSupplier leftTriggerSupplier;
  private DoubleSupplier rightTriggerSupplier;

  private double maximumDisplacement = 200;

  public WristControl(Wrist wrist, DoubleSupplier leftTriggerSupplier, DoubleSupplier rightTriggerSupplier) {
    this.wrist = wrist;
    this.leftTriggerSupplier = leftTriggerSupplier;
    this.rightTriggerSupplier = rightTriggerSupplier;

    addRequirements(wrist);
  }

  @Override
  public void execute() {
    double leftTrigger = this.leftTriggerSupplier.getAsDouble();
    double rightTrigger = this.rightTriggerSupplier.getAsDouble();
    double change = 0;

    if (rightTrigger != 0) {
      change += (maximumDisplacement * rightTrigger) / 50;
    }
    if (leftTrigger != 0) {
      change -= (maximumDisplacement * leftTrigger) / 50;
    }

    this.wrist.setWristSetpoint(this.wrist.getWristSetpoint() + change);
  }
}
