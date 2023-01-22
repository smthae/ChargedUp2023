package frc.robot.sysid;

import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SysIdSwerve extends SubsystemBase {
  public SysIdSwerveModule[] mSwerveMods;
  public Pigeon2 gyro;

  public SysIdSwerve() {
    /* Gyro setup */
    gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.pigeonCanBUS);
    gyro.configFactoryDefault();
    zeroGyro();

    /* Swerve modules setup */
    mSwerveMods = new SysIdSwerveModule[] {
        new SysIdSwerveModule(0, Constants.Swerve.Mod0.constants),
        new SysIdSwerveModule(1, Constants.Swerve.Mod1.constants),
        new SysIdSwerveModule(2, Constants.Swerve.Mod2.constants),
        new SysIdSwerveModule(3, Constants.Swerve.Mod3.constants)
    };
  }

  public void goToHome() {
    for (SysIdSwerveModule mod : mSwerveMods) {
      mod.goToHome();
    }
  }

  public void resetModuleZeros() {
    for (SysIdSwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(gyro.getPitch());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(gyro.getRoll());
  }

  public List<CANSparkMax> getLeftMotors() {
    return List.of(this.mSwerveMods[0].getDriveMotor(), this.mSwerveMods[2].getDriveMotor());
  }

  public List<CANSparkMax> getRightMotors() {
    return List.of(this.mSwerveMods[1].getDriveMotor(), this.mSwerveMods[3].getDriveMotor());
  }

  public double getGyroRate() {
    double[] xyz_change = new double[3];
    gyro.getRawGyro(xyz_change);

    return xyz_change[2];
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(this.mSwerveMods[0].getSpeed(), this.mSwerveMods[1].getSpeed());
  }

  public double getLeftDistanceMeters() {
    return this.mSwerveMods[0].getDistance();
  }

  public double getRightDistanceMeters() {
    return this.mSwerveMods[1].getDistance();
  }

}
