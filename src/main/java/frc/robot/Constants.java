package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.PIDConstants;
import frc.lib.util.SVAConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static enum RobotModes {
    Competition,
    Testing,
    SysID
  }

  public static final RobotModes robotMode = RobotModes.Competition;

  public static final class Vision {
    public static final String cameraName = "gloworm";
    public static final Transform3d cameraToRobot = new Transform3d(new Translation3d(-0.32, 0, 0), new Rotation3d());
    public static final Transform3d robotToCamera = cameraToRobot.inverse();
    public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    public static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    /* Custom PID Controllers for Vision */
    public static final ProfiledPIDController translationController = new ProfiledPIDController(3, 0, 0,
        TRANSLATION_CONSTRAINTS);
    public static final ProfiledPIDController rotationController = new ProfiledPIDController(2, 0, 0,
        ROTATION_CONSTRAINTS);
  }

  public static final class Operators {
    public static final int driver = 0;
  }

  public static final class Wrist {
    public static final int intakeMotorID = 9;
    public static final double power = 1;
  }

  public static final class Swerve {
    /* Drive Controls */
    public static final double stickDeadband = 0.1;

    /* Gyro */
    public static final int pigeonID = 21;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    public static final String pigeonCanBUS = "canivore3161";

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(23.0);
    public static final double wheelBase = Units.inchesToMeters(23.0);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final Matrix<N3, N1> stateStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05,
        Units.degreesToRadians(2));

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = 150.0 / 7.0; // 150/7:1

    /* Custom PID Controllers */
    public static final PIDConstants robotRotationPID = new PIDConstants(0.1, 0, 0.00005);
    public static final PIDConstants translationPID = new PIDConstants(4, 0, 0.005);
    public static final PIDConstants balancePID = new PIDConstants(0.01, 0, 0.005);

    /* Delays (milliseconds) */
    public static final long defenseDelay = 1000;

    /* Kinematics */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 40;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    public static final String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" }; // module #0,
                                                                                                           // #1, #2, #3

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 23;
      public static final String cancoderCANBUS = "canivore3161"; // change to "rio" if it's on rio
      public static final double angleOffset = 146.0;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final PIDConstants drivePID = new PIDConstants(0.1, 0.0, 0.000);
      public static final SVAConstants driveSVA = new SVAConstants(0.13522, 2.6797, 0.17176);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 24;
      public static final String cancoderCANBUS = "canivore3161";
      public static final double angleOffset = 324.0;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final PIDConstants drivePID = new PIDConstants(0.1, 0.0, 0.000);
      public static final SVAConstants driveSVA = new SVAConstants(0.13522, 2.6797, 0.17176);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 22;
      public static final String cancoderCANBUS = "canivore3161";
      public static final double angleOffset = 290.0;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final PIDConstants drivePID = new PIDConstants(0.1, 0.0, 0.000);
      public static final SVAConstants driveSVA = new SVAConstants(0.13522, 2.6797, 0.17176);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 25;
      public static final String cancoderCANBUS = "canivore3161";
      public static final double angleOffset = 90.0;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final PIDConstants drivePID = new PIDConstants(0.1, 0.0, 0.000);
      public static final SVAConstants driveSVA = new SVAConstants(0.13522, 2.6797, 0.17176);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final PIDConstants translationPID = new PIDConstants(0.2, 0, 0.05);
    public static final PIDConstants rotationPID = new PIDConstants(0.1, 0, 0.05);

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
