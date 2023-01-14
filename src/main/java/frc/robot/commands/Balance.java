package frc.robot.commands;

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
    double pitch = this.swerve.gyro.getPitch();
    double power = this.balanceController.calculate(pitch, 0);

    this.swerve.drive(new Translation2d(power, 0), 0, true, true, false, true, true);
  }

  @Override
  public boolean isFinished() {
    return this.balanceController.atSetpoint();
  }
}
