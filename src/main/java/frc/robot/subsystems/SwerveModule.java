package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

public class SwerveModule extends SubsystemBase {

  /**
   * Class to represent and handle a swerve module
   * A module's state is measured by a CANCoder for the absolute position,
   * integrated CANEncoder for relative position
   * for both rotation and linear movement
   */

  double shuffleboardTarget = 0;

  public double getNewTarget() {
    return shuffleboardTarget;
  }

  private static final double rotationkP = 0.75;
  private static final double rotationkD = 0.33;

  private static final double drivekP = 0.01;

  private final CANSparkMax driveMotor;
  private final CANSparkMax rotationMotor;

  public static double getRotationkp() {
    return rotationkP;
  }

  public static double getRotationkd() {
    return rotationkD;
  }

  public static double getDrivekp() {
    return drivekP;
  }

  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  public CANSparkMax getRotationMotor() {
    return rotationMotor;
  }

  public RelativeEncoder getDriveEncoder() {
    return driveEncoder;
  }

  public RelativeEncoder getRotationEncoder() {
    return rotationEncoder;
  }

  public CANCoder getCanCoder() {
    return canCoder;
  }

  public Rotation2d getOffset() {
    return offset;
  }

  public SparkMaxPIDController getRotationController() {
    return rotationController;
  }

  public SparkMaxPIDController getDriveController() {
    return driveController;
  }

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANCoder canCoder;

  // absolute offset for the CANCoder so that the wheels can be aligned when the
  // robot is turned on
  private final Rotation2d offset;

  private final SparkMaxPIDController rotationController;
  private final SparkMaxPIDController driveController;

  public SwerveModule(
      int driveMotorId,
      int rotationMotorId,
      int canCoderId,
      double measuredOffsetRadians) {

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    rotationEncoder = rotationMotor.getEncoder();

    canCoder = new CANCoder(canCoderId);

    offset = new Rotation2d(measuredOffsetRadians);

    driveMotor.setIdleMode(IdleMode.kBrake);
    rotationMotor.setIdleMode(IdleMode.kBrake);

    rotationController = rotationMotor.getPIDController();
    driveController = driveMotor.getPIDController();

    rotationController.setP(rotationkP);
    rotationController.setD(rotationkD);

    driveController.setP(drivekP);

    // set the output of the drive encoder to be in radians for linear measurement
    driveEncoder.setPositionConversionFactor(
        2.0 * Math.PI / Swerve.driveGearRatio);

    // set the output of the drive encoder to be in radians per second for velocity
    // measurement
    driveEncoder.setVelocityConversionFactor(
        2.0 * Math.PI / 60 / Swerve.driveGearRatio);

    // set the output of the rotation encoder to be in radians
    rotationEncoder.setPositionConversionFactor(2 * Math.PI / Swerve.angleGearRatio);

    // configure the CANCoder to output in unsigned (wrap around from 360 to 0
    // degrees)
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

  }

  public void resetDistance() {

    driveEncoder.setPosition(0.0);

  }

  public double getDriveDistanceRadians() {

    return driveEncoder.getPosition();

  }

  public Rotation2d getCanCoderAngle() {

    double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians())
        % (2 * Math.PI);

    return new Rotation2d(unsignedAngle);

  }

  public Rotation2d getIntegratedAngle() {

    double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

    if (unsignedAngle < 0)
      unsignedAngle += 2 * Math.PI;

    return new Rotation2d(unsignedAngle);

  }

  public double getCurrentVelocityRadiansPerSecond() {

    return driveEncoder.getVelocity();

  }

  public double getCurrentVelocityMetersPerSecond() {

    return driveEncoder.getVelocity() * (Swerve.wheelDiameter / 2.0);

  }

  // calculate the angle motor setpoint based on the desired angle and the current
  // angle measurement
  public static double placeInAppropriate0To360Scope(double targetAngle, double currentAngle) {

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

  // initialize the integrated NEO encoder to the offset (relative to home
  // position)
  // measured by the CANCoder
  public void initRotationOffset() {

    rotationEncoder.setPosition(-getCanCoderAngle().getRadians());

  }

  /**
   * Method to set the desired state of the swerve module
   * Parameter:
   * SwerveModuleState object that holds a desired linear and rotational setpoint
   * Uses PID and a feedforward to control the output
   */
  public void setDesiredStateClosedLoop(SwerveModuleState desiredState) {

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      driveMotor.set(0);
      rotationMotor.set(0);
      return;
    }

    SwerveModuleState optimizedDesiredState = OnboardModuleState.optimize(desiredState, getIntegratedAngle());
    rotationController.setReference(
        placeInAppropriate0To360Scope(optimizedDesiredState.angle.getRadians(), rotationEncoder.getPosition()),
        ControlType.kPosition);

    // rotationController.setReference(

    // state.angle.getRadians(),
    // ControlType.kPosition);

    // double speedRadPerSec = desiredState.speedMetersPerSecond /
    // (Swerve.wheelDiameter / 2);

    // driveController.setReference(
    // speedRadPerSec,
    // ControlType.kVelocity,
    // 0,
    // Swerve.driveFF.calculate(speedRadPerSec));

    driveMotor.set(optimizedDesiredState.speedMetersPerSecond / Swerve.maxSpeed);

  }

  public void resetEncoders() {

    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(0);

  }

}