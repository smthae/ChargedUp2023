// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}
