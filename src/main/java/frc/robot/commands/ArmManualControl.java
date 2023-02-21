package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmManualControl extends CommandBase {
  private Arm arm;
  private DoubleSupplier ySupplier;
  private double maximumDisplacement = 20; // 20 encoder ticks, might need to increase this if it's too slow

  public ArmManualControl(Arm arm, DoubleSupplier ySupplier) {
    this.arm = arm;
    this.ySupplier = ySupplier;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    double y = MathUtil.applyDeadband(this.ySupplier.getAsDouble(), Constants.Swerve.stickDeadband);
    double change = (maximumDisplacement * y) / 50;

    this.arm.setArmSetpoint(this.arm.getArmSetpoint() + change);
  }
}
