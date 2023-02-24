package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDConstants;
import frc.lib.util.ProfiledPIDConstants;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  // Motors
  private final CANSparkMax armLeader = new CANSparkMax(Constants.Arm.leaderMotorID, MotorType.kBrushless);
  private final CANSparkMax armFollower = new CANSparkMax(Constants.Arm.followerMotorID, MotorType.kBrushless);

  // Encoders
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Constants.Arm.encoderDIOPort);
  private final RelativeEncoder relativeEncoder;
  // Motion Control
  private final PIDConstants armPidConstants = Constants.Arm.armPID;
  private PIDController armRotationPID = Constants.Arm.armPID.getController();
  private SparkMaxPIDController armRotationController;

  private double armSetpoint = 0;


  public Arm() {
    /* Motors setup */

    // Arm leader
    this.armLeader.restoreFactoryDefaults();
    this.armLeader.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.armLeader.setSmartCurrentLimit(Constants.Arm.currentLimit);
    this.armLeader.setIdleMode(IdleMode.kBrake);

    // Arm follower
    this.armFollower.restoreFactoryDefaults();
    this.armFollower.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.armFollower.setSmartCurrentLimit(Constants.Arm.currentLimit);
    this.armFollower.follow(this.armLeader, true);
    this.armFollower.setIdleMode(IdleMode.kBrake);

    /* End Motors setup */

    // Encoder
    this.relativeEncoder = this.armLeader.getEncoder();
    if (this.absoluteEncoder.isConnected()) {
      this.relativeEncoder.setPosition(this.getEncoderPosition());
    } else {
      this.relativeEncoder.setPosition(Constants.Arm.encoderOffset - Units.degreesToRadians(45));
    }
    this.armSetpoint = this.getEncoderPosition();

    // PID
    SmartDashboard.putNumber("Arm rotation setpoint", 0);
    SmartDashboard.putNumber("Arm rotation encoder", 0);
    this.armRotationController = this.armLeader.getPIDController();
    this.armRotationController.setOutputRange(-Constants.Arm.armMaxOutput, Constants.Arm.armMaxOutput);
    this.armRotationPID.enableContinuousInput(-Math.PI, Math.PI);
    this.armPidConstants.applyPID(this.armRotationController);

    this.armPidConstants.sendDashboard("arm pid");
  }

  public double getEncoderPosition() {
    return Units.degreesToRadians(this.absoluteEncoder.getDistance() * (360 / 4));
  }

  public double getEncoderPositionWithOffset() {
    return this.getEncoderPosition() - Constants.Arm.encoderOffset;
  }

  /**
   * @param angle in degrees PLS
   */
  public void setArmSetpoint(double angle) {
    this.armSetpoint = Constants.Arm.encoderOffset - Units.degreesToRadians(90) + Units.degreesToRadians(angle);
  }

  public double getArmSetpoint() {
    return armSetpoint - Constants.Arm.encoderOffset + Units.degreesToRadians(90);
  }

  @Override
  public void periodic() {
    double pidOutput, feedforward;
    this.armPidConstants.retrieveDashboard(this.armRotationPID);
    this.armPidConstants.retrieveDashboard(this.armRotationController);

    this.armRotationController.setReference(this.armSetpoint, CANSparkMax.ControlType.kPosition);


    SmartDashboard.putNumber("Arm absolute encoder", Units.radiansToDegrees(this.getEncoderPosition()));
    SmartDashboard.putNumber("Arm absolute encoder with offset", Units.radiansToDegrees(this.getEncoderPosition()) - Units.radiansToDegrees(Constants.Arm.encoderOffset));
    SmartDashboard.putNumber("Arm pid output", this.armLeader.getAppliedOutput());
    SmartDashboard.putNumber("Arm rotation setpoint", Units.radiansToDegrees(this.getArmSetpoint()));
    this.armLeader.set(0);
  }
}
