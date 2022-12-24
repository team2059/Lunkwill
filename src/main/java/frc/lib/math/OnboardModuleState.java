package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveModule;

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
    double targetAngle = SwerveModule.placeInAppropriate0To360Scope(desiredState.angle.getRadians(),
        currentAngle.getRadians());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getRadians();
    if (Math.abs(delta) > Math.PI / 2) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > Math.PI / 2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
    }

    return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
  }
}