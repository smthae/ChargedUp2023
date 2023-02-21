package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class WristControl extends CommandBase {
  private Wrist wrist;
  private DoubleSupplier ySupplier;

  private double maximumDisplacement = 200;

  public WristControl(Wrist wrist, DoubleSupplier ySupplier) {
    this.wrist = wrist;
    this.ySupplier = ySupplier;

    addRequirements(wrist);
  }

  @Override
  public void execute() {
    double y = MathUtil.applyDeadband(this.ySupplier.getAsDouble(), Constants.Swerve.stickDeadband);

    double change = (maximumDisplacement * y) / 50;

    this.wrist.setWristSetpoint(this.wrist.getWristSetpoint() + change);
  }
}
