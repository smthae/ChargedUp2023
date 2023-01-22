// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.logging.SysIdDrivetrainLogger;
import frc.lib.logging.SysIdGeneralMechanismLogger;
import frc.lib.logging.SysIdMechanism;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(Constants.Operators.driver);

  /* Subsystems */
  final Swerve s_Swerve = new Swerve();
  final Wrist wrist = new Wrist();

  /* SysID logger */
  SysIdDrivetrainLogger sysidDrive;
  SysIdGeneralMechanismLogger sysidMech;
  SendableChooser<SubsystemBase> mechChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve.setName("Drive");
    s_Swerve.setDefaultCommand(new TeleopSwerve(
        s_Swerve,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX(),
        () -> driver.leftBumper().getAsBoolean(),
        () -> driver.rightBumper().getAsBoolean(),
        () -> driver.y().getAsBoolean(),
        () -> driver.b().getAsBoolean(),
        () -> driver.a().getAsBoolean(),
        () -> driver.x().getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();

    mechChooser = new SendableChooser<>();
    mechChooser.setDefaultOption("Drive", s_Swerve);
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
    /* Driver Buttons */
    driver.back().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    driver.start().whileTrue(new Balance(s_Swerve));
    driver.leftTrigger().and(() -> driver.rightTrigger().getAsBoolean()).whileTrue(new IntakeIn(wrist));
    driver.leftTrigger().whileTrue(new IntakeIn(wrist));
    driver.rightBumper().whileTrue(new IntakeIn(wrist));
    // perpendicular.onTrue(new PerpendicularTarget(s_Swerve));

    // snakeMode.toggleOnTrue(new SnakeSwerve(s_Swerve,
    // () -> -driver.getRawAxis(Constants.Swerve.translationAxis),
    // () -> -driver.getRawAxis(Constants.Swerve.strafeAxis),
    // () -> robotCentric.getAsBoolean(),
    // () -> driver.getRightBumper(),
    // () -> driver.getRawAxis(Constants.Swerve.NOS),
    // () -> faceForward.getAsBoolean(),
    // () -> faceRight.getAsBoolean(),
    // () -> faceBackwards.getAsBoolean(),
    // () -> faceLeft.getAsBoolean()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (Robot.sysidActive) {
      if (Robot.sysidActive) {
        if (mechChooser.getSelected().getName() != "Drive") {
          sysidMech = new SysIdGeneralMechanismLogger();
          SysIdMechanism mech = (SysIdMechanism) mechChooser.getSelected();
          return Commands.runOnce(sysidMech::initLogging, mechChooser.getSelected()).andThen(Commands.run(() -> {
            sysidMech.log(sysidMech.measureVoltage(List.of(mech.getMotor())), mech.getPosition(), mech.getVelocity());

            sysidMech.setMotorControllers(sysidMech.getMotorVoltage(), List.of(mech.getMotor()));
          }, mechChooser.getSelected()));
        }
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
    } else {
      return Commands.print("No auto selected!");
    }
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}
