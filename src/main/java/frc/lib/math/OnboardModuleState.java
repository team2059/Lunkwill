package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class OnboardModuleState {

  /**
   * Minimize the change in heading the desired swerve module state would require
   * by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to
   * include placing in
   * appropriate scope for CTRE and REV onboard control as both controllers as of
   * writing don't have
   * support for continuous input.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getRadians(), desiredState.angle.getRadians());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getRadians();
    if (Math.abs(delta) > Math.PI / 2) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > Math.PI / 2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
    }

    return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
  }

  // calculate the angle motor setpoint based on the desired angle and the current
  // angle measurement
  public static double placeInAppropriate0To360Scope(double currentAngle, double targetAngle) {

    double modAngle = currentAngle % (2.0 * Math.PI);

    if (modAngle < 0.0)
      modAngle += 2.0 * Math.PI;

    double newTarget = targetAngle + currentAngle - modAngle;

    if (targetAngle - modAngle > Math.PI)
      newTarget -= 2.0 * Math.PI;
    else if (targetAngle - modAngle < -Math.PI)
      newTarget += 2.0 * Math.PI;

    return newTarget;

  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle       Target Angle
   * @return Closest angle within scope
   */
  // private static double placeInAppropriate0To360Scope(double scopeReference,
  // double newAngle) {
  // double lowerBound;
  // double upperBound;
  // double lowerOffset = scopeReference % 360;
  // if (lowerOffset >= 0) {
  // lowerBound = scopeReference - lowerOffset;
  // upperBound = scopeReference + (360 - lowerOffset);
  // } else {
  // upperBound = scopeReference - lowerOffset;
  // lowerBound = scopeReference - (360 + lowerOffset);
  // }
  // while (newAngle < lowerBound) {
  // newAngle += 360;
  // }
  // while (newAngle > upperBound) {
  // newAngle -= 360;
  // }
  // if (newAngle - scopeReference > 180) {
  // newAngle -= 360;
  // } else if (newAngle - scopeReference < -180) {
  // newAngle += 360;
  // }
  // return newAngle;
  // }
}
