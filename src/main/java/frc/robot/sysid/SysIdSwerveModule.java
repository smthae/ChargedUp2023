package frc.robot.sysid;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.PIDConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SysIdSwerveModule {
  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  /* Encoders and their values */
  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;
  private double angleOffset;

  /* Controllers */
  public final SparkMaxPIDController driveController;
  public final SparkMaxPIDController angleController;
  public final PIDConstants anglePID;

  public SysIdSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    angleOffset = moduleConstants.angleOffset;
    this.anglePID = moduleConstants.anglePID;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID, moduleConstants.cancoderCANBUS);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

  }

  public void resetToAbsolute() {
    double integratedAngleEncoderPosition = this.integratedAngleEncoder.getPosition();
    double absolutePosition = integratedAngleEncoderPosition - integratedAngleEncoderPosition % 360
        + (getCanCoder().getDegrees() - angleOffset);
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kSensorDataOnly);
    angleEncoder.configAllSettings(SysIdRobot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    this.anglePID.applyPID(this.angleController);
    angleController.setFF(0);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setFF(0);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveEncoder.setPosition(0.0);
  }

  public void goToHome() {
    Rotation2d angle = getAngle();
    angleController.setReference(angle.getDegrees() - angle.getDegrees() % 360,
        ControlType.kPosition);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(this.angleEncoder.getAbsolutePosition());
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(this.integratedAngleEncoder.getPosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getSpeed(), this.getAngle());
  }

  public double getSpeed() {
    return this.driveEncoder.getVelocity();
  }

  public double getDistance() {
    return this.driveEncoder.getPosition() * Constants.Swerve.wheelCircumference;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.getDistance(), this.getAngle());
  }

  public CANSparkMax getDriveMotor() {
    return this.driveMotor;
  }
}
