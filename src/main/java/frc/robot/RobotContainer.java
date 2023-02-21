// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Hashtable;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PieceType;
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
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Subsystems */
  final PhotonCamera camera = new PhotonCamera(Constants.Vision.cameraName);
  final Swerve s_Swerve = new Swerve(camera);
  final Wrist wrist = new Wrist();
  final Arm arm = new Arm();
  public final PoseEstimator poseEstimator = new PoseEstimator(s_Swerve, camera);

  /* Auto */
  Hashtable<String, AutoImpl> autoCommands = new Hashtable<String, AutoImpl>();

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

    wrist.setDefaultCommand(
        new WristControl(wrist, () -> operator.getRightY()));

    arm.setDefaultCommand(
        new ArmManualControl(arm, () -> operator.getLeftY()));

    // Configure the button bindings
    configureButtonBindings();
    configureAutoCommands();
    configureTestCommands();
    sendAutoCommands();
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
    driver.leftTrigger().and(() -> driver.rightTrigger().getAsBoolean()).whileTrue(new IntakeIn(wrist, this.s_Swerve));
    driver.leftTrigger().whileTrue(new IntakeIn(wrist, this.s_Swerve));
    driver.rightTrigger().whileTrue(new IntakeOut(wrist));

    operator.y().onTrue(new InstantCommand(() -> {
      wrist.setWristSetpoint(180);
      wrist.currentPiece = PieceType.CUBE;
    }));
    operator.a().onTrue(new InstantCommand(() -> {
      wrist.setWristSetpoint(15);
      wrist.currentPiece = PieceType.CONE;
    }));
    operator.back().onTrue(new InstantCommand(() -> wrist.resetWristEncoder()));
    operator.start().onTrue(new InstantCommand(() -> arm.resetArmEncoder()));

  }

  public void configureAutoCommands() {
    this.autoCommands.put("1 cone sus", new exampleAuto(s_Swerve, camera, poseEstimator, wrist));
    this.autoCommands.put("Example auto 2", new exampleAuto2(s_Swerve, camera, poseEstimator));
    this.autoCommands.put("Example auto 3", new exampleAuto3(s_Swerve, camera, poseEstimator));
  }

  public void configureTestCommands() {
    SmartDashboard.putData("Reset Pose Estimator", new InstantCommand(() -> {
      this.poseEstimator.resetFieldPosition();
    }));
    GoToPosition goHome = new GoToPosition(s_Swerve, poseEstimator,
        new Transform3d(new Translation3d(), new Rotation3d()));
    goHome.setDebug(true);

    SmartDashboard.putData("Go to home", goHome);
  }

  public void sendAutoCommands() {
    SmartDashboard.putString("selectedAuto", "");
    JSONObject autoJSON = new JSONObject();

    this.autoCommands.forEach((key, value) -> {
      JSONArray translation = new JSONArray();
      Pose2d initialHolonomicPose = value.getInitialHolonomicPose();
      translation.add(initialHolonomicPose.getX());
      translation.add(initialHolonomicPose.getY());

      autoJSON.put(key, translation);
    });

    SmartDashboard.putString("autos", autoJSON.toJSONString());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    String selectedAuto = SmartDashboard.getString("selectedAuto", "default");
    if (selectedAuto.equals("default") || !this.autoCommands.containsKey(selectedAuto)) {
      return new DefaultAuto(this.wrist);
    }

    return this.autoCommands.get(selectedAuto).getCommand();
  }
}
