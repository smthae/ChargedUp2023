// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.util.PIDConstants;
// import frc.lib.util.ProfiledPIDConstants;
// import frc.robot.Constants;

// public class armold extends SubsystemBase {
// // Motors
// private final CANSparkMax armLeader = new
// CANSparkMax(Constants.Arm.leaderMotorID, MotorType.kBrushless);
// private final CANSparkMax armFollower = new
// CANSparkMax(Constants.Arm.followerMotorID, MotorType.kBrushless);

// // Encoders
// private final RelativeEncoder armEncoder;
// private final DutyCycleEncoder absoluteEncoder = new
// DutyCycleEncoder(Constants.Arm.encoderDIOPort);

// // Motion Control
// private final ProfiledPIDConstants armPidConstants = Constants.Arm.armPID;
// private ProfiledPIDController armRotationPID =
// Constants.Arm.armPID.getController();
// private double armSetPoint = 0;

// public armold() {
// /* Motors setup */

// // Arm leader
// this.armLeader.restoreFactoryDefaults();
// this.armLeader.enableVoltageCompensation(Constants.Swerve.voltageComp);
// this.armLeader.setSmartCurrentLimit(Constants.Arm.currentLimit);
// this.armLeader.setIdleMode(IdleMode.kBrake);

// // Arm follower
// this.armFollower.restoreFactoryDefaults();
// this.armFollower.enableVoltageCompensation(Constants.Swerve.voltageComp);
// this.armFollower.setSmartCurrentLimit(Constants.Arm.currentLimit);
// this.armFollower.follow(this.armLeader, true);
// this.armFollower.setIdleMode(IdleMode.kBrake);

// /* End Motors setup */

// // Encoder
// this.armEncoder = this.armLeader.getEncoder();
// this.armEncoder.setPositionConversionFactor(360 / Constants.Arm.gearRatio);
// this.absoluteEncoder.setPositionOffset(Constants.Arm.encoderOffset / 360);
// if (this.absoluteEncoder.isConnected()) {
// this.armEncoder.setPosition(this.absoluteEncoder.getDistance() * 360 -
// Constants.Arm.encoderOffset);
// } else {
// this.armEncoder.setPosition(0);
// }

// // PID
// SmartDashboard.putNumber("Arm rotation setpoint", 0);
// SmartDashboard.putNumber("Arm rotation encoder", 0);
// this.armRotationPID.enableContinuousInput(-180, 180);

// this.armPidConstants.sendDashboard("arm pid");
// }

// public void resetArmEncoder() {
// this.armEncoder.setPosition(0);
// this.setArmSetpoint(0);

// // CustomThreads.setTimeout(() -> {
// // this.armSetPoint = 0;
// // }, 20);
// }

// public void setArmSetpoint(double angle) {
// this.armSetPoint = angle;
// }

// public double getArmSetpoint() {
// return this.armSetPoint;
// }

// @Override
// public void periodic() {
// double pidOutput, feedforward;
// this.armPidConstants.retrieveDashboard(this.armRotationPID);

// if (this.absoluteEncoder.isConnected()) {
// pidOutput = this.armRotationPID.calculate(this.absoluteEncoder.getDistance()
// * 360 + Constants.Arm.encoderOffset);
// feedforward =
// Constants.Arm.armFF.calculate(this.absoluteEncoder.getDistance() * 360, 0);
// SmartDashboard.putNumber("Arm absolute encoder",
// this.absoluteEncoder.getDistance() * 360);
// } else {
// pidOutput = this.armRotationPID.calculate(this.armEncoder.getPosition());
// feedforward = Constants.Arm.armFF.calculate(this.armEncoder.getPosition(),
// 0);
// }

// SmartDashboard.putNumber("Arm pid output", pidOutput);
// SmartDashboard.putNumber("Arm rotation encoder",
// this.armEncoder.getPosition());
// SmartDashboard.putNumber("Arm rotation setpoint", this.armSetPoint);

// if (!this.armRotationPID.atSetpoint()) {
// this.armLeader.set(MathUtil.clamp(pidOutput, -0.3, 0.3));

// }
// // this.armLeader.set(pidOutput + feedforward);
// }
// }
