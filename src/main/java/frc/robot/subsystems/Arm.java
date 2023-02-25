package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  // Motion Control
  private final PIDConstants armPidConstants = Constants.Arm.armPID;
  private PIDController armRotationPID = Constants.Arm.armPID.getController();

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
    this.armSetpoint = this.getEncoderPosition();

    // PID
    SmartDashboard.putNumber("Arm rotation setpoint", 0);
    SmartDashboard.putNumber("Arm rotation encoder", 0);
    this.armRotationPID.enableContinuousInput(-180, 180);

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

    pidOutput = MathUtil.clamp(this.armRotationPID
        .calculate(this.getEncoderPosition(), this.armSetpoint), -0.05,
        Constants.Arm.armMaxOutput);

    feedforward = Constants.Arm.armFF.calculate(
        this.getEncoderPosition() - Constants.Arm.encoderOffset, 0);

    SmartDashboard.putNumber("Arm absolute encoder", Units.radiansToDegrees(this.getEncoderPosition()));
    SmartDashboard.putNumber("Arm absolute encoder with offset",
        Units.radiansToDegrees(this.getEncoderPosition()) - Units.radiansToDegrees(Constants.Arm.encoderOffset));
    SmartDashboard.putNumber("Arm pid output", pidOutput);
    SmartDashboard.putNumber("Arm pid + ff", pidOutput + feedforward);
    SmartDashboard.putNumber("Arm rotation setpoint", Units.radiansToDegrees(this.getArmSetpoint()));
    SmartDashboard.putNumber("error", this.getEncoderPosition() - this.armSetpoint);
    // this.armLeader.set(MathUtil.clamp(pidOutput, -0.3, 0.3));
    this.armLeader.set(pidOutput);
  }
}
