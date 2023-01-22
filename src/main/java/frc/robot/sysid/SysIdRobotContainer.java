// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.SysIdDrivetrainLogger;
import frc.lib.logging.SysIdGeneralMechanismLogger;
import frc.lib.logging.SysIdMechanism;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link SysIdRobot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SysIdRobotContainer {

  final SysIdSwerve s_Swerve = new SysIdSwerve();

  /* SysID logger */
  SysIdDrivetrainLogger sysidDrive;
  SysIdGeneralMechanismLogger sysidMech;
  SendableChooser<SubsystemBase> mechChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public SysIdRobotContainer() {
    s_Swerve.setName("Drive");
    // Configure the button bindings
    configureButtonBindings();

    mechChooser = new SendableChooser<>();
    mechChooser.setDefaultOption("Drive", s_Swerve);
    SmartDashboard.putData(mechChooser);
  }

  /** Actions that we want to do when the robot is disabled. */
  public void disabledActions() {
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link SysIdRobot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (!mechChooser.getSelected().getName().equals("Drive")) {
      sysidMech = new SysIdGeneralMechanismLogger();
      SysIdMechanism mech = (SysIdMechanism) mechChooser.getSelected();
      return Commands.runOnce(sysidMech::initLogging,
          mechChooser.getSelected()).andThen(Commands.run(() -> {
            sysidMech.log(sysidMech.measureVoltage(List.of(mech.getMotor())),
                mech.getPosition(), mech.getVelocity());

            sysidMech.setMotorControllers(sysidMech.getMotorVoltage(),
                List.of(mech.getMotor()));
          }, mechChooser.getSelected()));
    } else {
      sysidDrive = new SysIdDrivetrainLogger();
      return Commands.runOnce(sysidDrive::initLogging, this.s_Swerve).andThen(
          Commands.run(() -> {
            sysidDrive.log(
                sysidDrive.measureVoltage(this.s_Swerve.getLeftMotors()),
                sysidDrive.measureVoltage(this.s_Swerve.getRightMotors()),
                this.s_Swerve.getLeftDistanceMeters(),
                this.s_Swerve.getRightDistanceMeters(),
                this.s_Swerve.getWheelSpeeds().leftMetersPerSecond,
                this.s_Swerve.getWheelSpeeds().rightMetersPerSecond,
                this.s_Swerve.getYaw().getDegrees(),
                this.s_Swerve.getGyroRate());
            sysidDrive.setMotorControllers(sysidDrive.getLeftMotorVoltage(), this.s_Swerve.getLeftMotors());
            sysidDrive.setMotorControllers(sysidDrive.getRightMotorVoltage(), this.s_Swerve.getRightMotors());
          }, this.s_Swerve));
    }
  }
}
