package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;

public class Wrist extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(Constants.Wrist.intakeMotorID);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private double power = Constants.Wrist.power;

  public Wrist() {
    this.intakeMotor.configVoltageCompSaturation(Constants.Swerve.voltageComp);
    this.intakeMotor.enableVoltageCompensation(true);
    SmartDashboard.putNumber("wrist power", this.power);

    TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();
    intakeMotorConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        true,
        7,
        8,
        0.1);

    intakeMotor.configFactoryDefault();
    intakeMotor.configAllSettings(intakeMotorConfiguration);
  }

  public void updatePower() {
    double newPower = SmartDashboard.getNumber("wrist power", power);
    if (newPower != this.power) {
      this.power = newPower;
    }
  }

  public void intakeIn() {
    this.updatePower();
    this.intakeMotor.set(ControlMode.PercentOutput, -this.power);
  }

  public void intakeOut() {
    this.updatePower();
    this.intakeMotor.set(ControlMode.PercentOutput, this.power);
  }

  public void intakeStop() {
    this.intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public PieceType getGamPieceType() {
    Color detectedColor = colorSensor.getColor();
    PieceType output;

    int proximity = colorSensor.getProximity();
    if (proximity < 55) {
      output = PieceType.AIR;
    } else {
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Proximity", proximity);
      if (detectedColor.red > 0.17 && detectedColor.red < 0.33 && detectedColor.green > 0.27
          && detectedColor.green < 0.48
          && detectedColor.blue < 0.49 && detectedColor.blue > 0.27) {
        output = PieceType.CUBE;
      } else if (detectedColor.red > 0.31 && detectedColor.red < 0.40 && detectedColor.green > 0.45
          && detectedColor.green < 0.55 && detectedColor.blue > 0 && detectedColor.blue < 0.23) {
        output = PieceType.CONE;
      } else {
        output = PieceType.AIR;
      }
    }
    return output;
  }

  @Override
  public void periodic() {
    PieceType gamePieceType = this.getGamPieceType();

    switch (gamePieceType) {
      case AIR:
        SmartDashboard.putString("color sensor", "Nothing - AIR");
        break;

      case CONE:
        SmartDashboard.putString("color sensor", "CONE");
        break;
      case CUBE:
        SmartDashboard.putString("color sensor", "CUBE");
        break;
      default:
        break;
    }
  }
}
