package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase {
  private final Swerve swerve;
  private final PIDController balanceController = Constants.Swerve.balancePID.getController();

  public Balance(Swerve swerve) {
    this.swerve = swerve;

    this.addRequirements(swerve);
  }

  @Override
  public void execute() {
    double pitch = this.swerve.getPitch().getDegrees();
    double power = MathUtil.clamp(this.balanceController.calculate(pitch, 0), -1, 1);

    this.swerve.drive(new Translation2d(power, 0).times(Constants.Swerve.maxSpeed), 0, true, true, false, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.brake();
    
  }

  @Override
  public boolean isFinished() {
    return this.balanceController.atSetpoint();
  }
}
