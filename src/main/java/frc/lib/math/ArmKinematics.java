package frc.lib.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class ArmKinematics {
  private final double armLength;
  private final double wristLength;
  private Translation2d position;
  private double wristAngle = 0;

  public ArmKinematics(double armLength, double wristLength) {
    this.armLength = armLength;
    this.wristLength = wristLength;
  }

  public void setPosition(Translation2d position) {
    this.position = position;
  }

  public Translation2d getPosition() {
    return position;
  }

  /**
   * @param wristAngle Wrist angle in radians relative to the ground
   */
  public void setWristAngle(double wristAngle) {
    this.wristAngle = Units.degreesToRadians(wristAngle);
  }

  public double getWristAngle() {
    return wristAngle;
  }

  private Translation2d getWristTranslation() {
    double x = Math.cos(this.wristAngle) * this.wristLength;
    double y = Math.sin(this.wristAngle) * this.wristLength;

    return new Translation2d(x, y);
  }

  public double[] calculate() {
    Translation2d wristTranslation = this.getWristTranslation();
    Translation2d armTipPosition = this.position.minus(wristTranslation);

    double armPivotAngle = Math.asin(armTipPosition.getY() / this.armLength);
    double wristAngle = Math.PI - this.wristAngle - armPivotAngle;

    return new double[]{armPivotAngle, wristAngle};
  }
}
