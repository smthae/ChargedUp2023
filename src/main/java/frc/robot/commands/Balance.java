package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.PIDConstants;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase {
  private final Swerve swerve;
  private final PIDConstants balanceControllerC = Constants.Swerve.balancePID;
  private  PIDController balanceController = Constants.Swerve.balancePID.getController();
  private final LEDs leds;

  public Balance(Swerve swerve, LEDs leds) {
    this.swerve = swerve;
    this.leds = leds;
    this.balanceControllerC.sendDashboard("balance");
    this.addRequirements(swerve);
  }

  @Override
  public void execute() {
    double yaw = this.swerve.getYaw().getDegrees();
    double power = 0;

    power = -MathUtil.clamp(this.balanceController.calculate(this.swerve.getRoll().getDegrees(), 0), -0.2, 0.2);
    // if (yaw <= 45 && 360 - yaw >= 315) {
    // this.swerve.setHold(0);
    // power =
    // -MathUtil.clamp(this.balanceController.calculate(this.swerve.getRoll().getDegrees(),
    // 0), -0.2, 0.2);
    // } else if (yaw > 45 && yaw <= 135) {
    // this.swerve.setHold(90);
    // power =
    // -MathUtil.clamp(this.balanceController.calculate(this.swerve.getPitch().getDegrees(),
    // 0), -0.2, 0.2);
    // } else if (yaw > 135 && yaw <= 225) {
    // this.swerve.setHold(180);
    // power =
    // -MathUtil.clamp(this.balanceController.calculate(this.swerve.getRoll().getDegrees(),
    // 0), -0.2, 0.2);
    // } else if (yaw > 225 && yaw < 315) {
    // this.swerve.setHold(270);
    // power =
    // -MathUtil.clamp(this.balanceController.calculate(this.swerve.getPitch().getDegrees(),
    // 0), -0.2, 0.2);
    // }

    this.swerve.drive(new Translation2d(power, 0), 0, true, true, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.brake();
  }

  @Override
  public boolean isFinished() {
    this.leds.set(Constants.LEDConstants.off);
    if (this.balanceController.atSetpoint()) {
      this.leds.set(Constants.LEDConstants.raindbow);
      return true;
    }

    return false;
  }
}