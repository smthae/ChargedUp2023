package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(Constants.Wrist.intakeMotorID);

  public Wrist() {
  }

  public void intakeIn() {
    this.intakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.power);
  }

  public void intakeOut() {
    this.intakeMotor.set(ControlMode.PercentOutput, -Constants.Wrist.power);
  }

  public void intakeStop() {
    this.intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}
