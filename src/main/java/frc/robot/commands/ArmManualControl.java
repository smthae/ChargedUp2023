package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class ArmManualControl extends CommandBase {
  private Arm arm;
  private Wrist wrist;
  private DoubleSupplier armYSupplier;
  private DoubleSupplier wristYSupplier;
  private double maximumDisplacement = 20; // 20 degrees, might need to increase this if it's too slow

  public ArmManualControl(Arm arm, Wrist wrist, DoubleSupplier armYSupplier, DoubleSupplier wristYSupplier) {
    this.arm = arm;
    this.wrist = wrist;
    this.armYSupplier = armYSupplier;
    this.wristYSupplier = wristYSupplier;

    addRequirements(arm, wrist);
  }

  @Override
  public void execute() {
    // Arm
    double armY = MathUtil.applyDeadband(this.armYSupplier.getAsDouble(), Constants.Swerve.stickDeadband);
    double armChange = (armY  * maximumDisplacement) / 50;

    this.arm.setArmSetpoint(this.arm.getArmSetpoint() + armChange);

    // Wrist
    double wristY = MathUtil.applyDeadband(this.wristYSupplier.getAsDouble(), Constants.Swerve.stickDeadband);
    double wristChange = (wristY * maximumDisplacement) / 50;

    this.wrist.setWristSetpoint(this.wrist.getWristSetpoint() + wristChange);
  }
}
