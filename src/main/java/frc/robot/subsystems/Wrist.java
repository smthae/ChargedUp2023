package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(Constants.Wrist.intakeMotorID);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  public Wrist() {
    this.intakeMotor.configVoltageCompSaturation(Constants.Swerve.voltageComp);
    this.intakeMotor.enableVoltageCompensation(true);
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

  @Override
  public void periodic() {
    Color detectedColor = colorSensor.getColor();
    int proximity = colorSensor.getProximity();
    if (proximity < 55) {
      SmartDashboard.putBoolean("CONE", false);
      SmartDashboard.putBoolean("CUBE", false);
      return;
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Proximity", proximity);
    if (detectedColor.red > 0.17 && detectedColor.red < 0.33 && detectedColor.green > 0.27 && detectedColor.green < 0.48
        && detectedColor.blue < 0.49 && detectedColor.blue > 0.27) {
      SmartDashboard.putBoolean("CUBE", true);
      SmartDashboard.putBoolean("CONE", false);

    } else if (detectedColor.red > 0.31 && detectedColor.red < 0.40 && detectedColor.green > 0.45
        && detectedColor.green < 0.55 && detectedColor.blue > 0 && detectedColor.blue < 0.23) {
      SmartDashboard.putBoolean("CONE", true);
      SmartDashboard.putBoolean("CUBE", false);

    } else {
      SmartDashboard.putBoolean("CONE", false);
      SmartDashboard.putBoolean("CUBE", false);
    }
  }
}
