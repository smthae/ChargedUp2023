package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CustomThreads;
import frc.lib.util.PIDConstants;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;

public class Wrist extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(Constants.Wrist.intakeMotorID);
  private double power = Constants.Wrist.power;

  private final CANSparkMax wristMotor = new CANSparkMax(Constants.Wrist.wristMotorID, MotorType.kBrushless);
  private final RelativeEncoder wristEncoder;
  private final SparkMaxPIDController wristController;
  private final PIDConstants wristRotationPID = Constants.Wrist.wristRotationPID;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private double wristSetPoint = 0;
  public PieceType currentPiece = PieceType.AIR;

  public Wrist() {
    this.intakeMotor.configVoltageCompSaturation(Constants.Swerve.voltageComp);
    this.intakeMotor.enableVoltageCompensation(true);
    SmartDashboard.putNumber("wrist power", this.power);

    this.wristMotor.restoreFactoryDefaults();
    this.wristMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.wristMotor.setSmartCurrentLimit(30);
    this.wristEncoder = this.wristMotor.getEncoder();
    this.wristEncoder.setPositionConversionFactor(Constants.Wrist.wristGearRatio);
    this.wristEncoder.setPosition(0);
    this.wristController = this.wristMotor.getPIDController();
    this.wristRotationPID.applyPID(this.wristController);
    this.wristController.setOutputRange(-0.08, 0.08);

    this.wristRotationPID.sendDashboard("Wrist Rotation");
    SmartDashboard.putNumber("Wrist rotation setpoint", 0);
    SmartDashboard.putNumber("Wrist rotation encoder", 0);

    TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();
    // intakeMotorConfiguration.supplyCurrLimit = new
    // SupplyCurrentLimitConfiguration(
    // true,
    // 20,
    // 20,
    // 0.1);

    intakeMotor.configFactoryDefault();
    intakeMotor.configAllSettings(intakeMotorConfiguration);
  }

  public void updatePower() {
    double newPower = SmartDashboard.getNumber("wrist power", power);
    if (newPower != this.power) {
      this.power = newPower;
    }
  }

  public void intakeIn(PieceType gamePiece) {
    this.updatePower();
    if (gamePiece == PieceType.CONE) {
      this.intakeMotor.set(ControlMode.PercentOutput, -this.power);
    } else if (gamePiece == PieceType.CUBE) {
      this.intakeMotor.set(ControlMode.PercentOutput, this.power);
    }
  }

  public void intakeOut(PieceType gamePiece) {
    this.updatePower();
    if (gamePiece == PieceType.CONE) {
      this.intakeMotor.set(ControlMode.PercentOutput, this.power);
    } else if (gamePiece == PieceType.CUBE) {
      this.intakeMotor.set(ControlMode.PercentOutput, -this.power);
    }
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

  public void setWristSetpoint(double value) {
    this.wristSetPoint = value;
  }

  public double getWristSetpoint() {
    return this.wristSetPoint;
  }

  public void resetWristEncoder() {
    this.wristEncoder.setPosition(0);
    this.wristSetPoint = 0;

    // CustomThreads.setTimeout(() -> {
    // this.wristSetPoint = 0;
    // }, 20);
  }

  @Override
  public void periodic() {
    this.wristRotationPID.retrieveDashboard(this.wristController);

    SmartDashboard.putNumber("Wrist rotation encoder", this.wristEncoder.getPosition());
    this.wristController.setReference(this.wristSetPoint, ControlType.kPosition);

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
