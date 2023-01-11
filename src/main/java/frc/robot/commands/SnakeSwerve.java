package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Swerve;

public class SnakeSwerve extends TeleopSwerve {

  public SnakeSwerve(
      Swerve swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier rightBumper,
      DoubleSupplier NOSMode,
      BooleanSupplier faceForward,
      BooleanSupplier faceRight,
      BooleanSupplier faceBackwards,
      BooleanSupplier faceLeft,
      DoubleSupplier DPad) {
    super(swerve, translationSup, strafeSup, (DoubleSupplier) () -> 0, robotCentricSup,
        rightBumper, NOSMode,
        faceForward, faceRight,
        faceBackwards, faceLeft, DPad);
  }

  @Override
  public double pointTo() {
    double angle = super.pointTo();
    if (angle != -1)
      return angle;

    double[] joystickValues = this.getJoystickValues();
    double translationVal = joystickValues[0];
    double strafeVal = joystickValues[1];

    double rotationDegree = Math.atan(strafeVal / translationVal);
    this.swerve.setHold(rotationDegree);
    return rotationDegree;
  }
}
