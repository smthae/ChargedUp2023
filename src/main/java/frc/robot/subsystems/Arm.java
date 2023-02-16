package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDConstants;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  // Motors
  private final CANSparkMax armLeader = new CANSparkMax(Constants.Arm.leaderMotorID, MotorType.kBrushless);
  private final CANSparkMax armFollower = new CANSparkMax(Constants.Arm.followerMotorID, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder armEncoder;

  // Motion Control
  private final SparkMaxPIDController armController;
  private final PIDConstants armRotationPID = Constants.Arm.armPID;
  public double armSetPoint = 0;

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
    this.armLeader.setIdleMode(IdleMode.kBrake);

    /* End Motors setup */

    // Encoder
    this.armEncoder = this.armLeader.getEncoder();
    this.armEncoder.setPositionConversionFactor(Constants.Arm.gearRatio);
    this.armEncoder.setPosition(0);

    // PID
    this.armController = this.armLeader.getPIDController();
    this.armRotationPID.applyPID(armController);
    this.armController.setOutputRange(-Constants.Arm.armMaxOutput, Constants.Arm.armMaxOutput);

    this.armRotationPID.sendDashboard("Arm rotation");
    SmartDashboard.putNumber("Arm rotation setpoint", 0);
    SmartDashboard.putNumber("Arm rotation encoder", 0);
  }

  public void resetArmEncoder() {
    this.armEncoder.setPosition(0);
    this.armSetPoint = 0;

    // CustomThreads.setTimeout(() -> {
    // this.armSetPoint = 0;
    // }, 20);
  }

  @Override
  public void periodic() {
    this.armRotationPID.retrieveDashboard(this.armController);

    SmartDashboard.putNumber("Wrist rotation encoder", this.armEncoder.getPosition());
    this.armController.setReference(this.armSetPoint, ControlType.kPosition);

  }
}
