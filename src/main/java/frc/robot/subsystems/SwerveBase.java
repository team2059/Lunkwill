package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveBase extends SubsystemBase {

  /**
   * Subsystem that controls the drivetrain of the robot
   * Handles all the odometry and base movement for the chassis
   */

  /**
   * absolute encoder offsets for the wheels
   * 180 degrees added to offset values to invert one side of the robot so that it
   * doesn't spin in place
   */
  private static final double frontLeftAngleOffset = Units.degreesToRadians(284.117);
  private static final double frontRightAngleOffset = Units.degreesToRadians(285.996 - 61);
  private static final double rearLeftAngleOffset = Units.degreesToRadians(137 + 134);
  private static final double rearRightAngleOffset = Units.degreesToRadians(232.229 + 2);

  /**
   * SwerveModule objects
   * Parameters:
   * drive motor can ID
   * rotation motor can ID
   * external CANCoder can ID
   * measured CANCoder offset
   */

  private final SwerveModule frontLeft = new SwerveModule(
      Swerve.frontLeftDriveMotorId,
      Swerve.frontLeftRotationMotorId,
      Swerve.frontLeftRotationEncoderId,
      frontLeftAngleOffset);

  public static double getFrontleftangleoffset() {
    return frontLeftAngleOffset;
  }

  public static double getFrontrightangleoffset() {
    return frontRightAngleOffset;
  }

  public static double getRearleftangleoffset() {
    return rearLeftAngleOffset;
  }

  public static double getRearrightangleoffset() {
    return rearRightAngleOffset;
  }

  public SwerveModule getFrontLeft() {
    return frontLeft;
  }

  public SwerveModule getFrontRight() {
    return frontRight;
  }

  public SwerveModule getRearLeft() {
    return rearLeft;
  }

  public SwerveModule getRearRight() {
    return rearRight;
  }

  private final SwerveModule frontRight = new SwerveModule(
      Swerve.frontRightDriveMotorId,
      Swerve.frontRightRotationMotorId,
      Swerve.frontRightRotationEncoderId,
      frontRightAngleOffset);

  private final SwerveModule rearLeft = new SwerveModule(
      Swerve.rearLeftDriveMotorId,
      Swerve.rearLeftRotationMotorId,
      Swerve.rearLeftRotationEncoderId,
      rearLeftAngleOffset);

  private final SwerveModule rearRight = new SwerveModule(
      Swerve.rearRightDriveMotorId,
      Swerve.rearRightRotationMotorId,
      Swerve.rearRightRotationEncoderId,
      rearRightAngleOffset);

  // commanded values from the joysticks and field relative value to use in
  // AlignWithTargetVision and AlignWithGyro
  private double commandedForward = 0;
  private double commandedStrafe = 0;
  private double commandedRotation = 0;

  private boolean isCommandedFieldRelative = false;

  private final AHRS navX = new AHRS(SPI.Port.kMXP);

  /**
   * odometry for the robot, measured in meters for linear motion and radians for
   * rotational motion
   * Takes in kinematics and robot angle for parameters
   */
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      Swerve.kinematics,
      new Rotation2d(getHeading().getRadians()));

  public SwerveBase() {

    // wait 1 second for navX to calibrate
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetImu();
      } catch (Exception e) {
      }

    }).start();

    // initialize the rotation offsets for the CANCoders
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    rearLeft.initRotationOffset();
    rearRight.initRotationOffset();

    // reset the measured distance driven for each module
    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

    rearLeft.getDriveMotor().setInverted(false);
    frontRight.getDriveMotor().setInverted(false);
    frontLeft.getDriveMotor().setInverted(false);

  }

  @Override
  public void periodic() {

    // update the odometry every 20ms
    odometry.update(getHeading(), getModuleStates());

    SmartDashboard.putNumber("heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Odometry x", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry y", odometry.getPoseMeters().getY());

    SmartDashboard.putNumber("CAN FL",
        frontLeft.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("CAN FR",
        frontRight.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("CAN RL", rearLeft.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("CAN RR", rearRight.getCanCoderAngle().getDegrees());

    SmartDashboard.putNumber("SPARK FL",
        frontLeft.getIntegratedAngle().getDegrees());
    SmartDashboard.putNumber("SPARK FR",
        frontRight.getIntegratedAngle().getDegrees());
    SmartDashboard.putNumber("SPARK RL",
        rearLeft.getIntegratedAngle().getDegrees());
    SmartDashboard.putNumber("SPARK RR", rearRight.getIntegratedAngle().getDegrees());

    SmartDashboard.putNumber("FL setpoint", frontLeft.getNewTarget());
    SmartDashboard.putNumber("FR setpoint", frontRight.getNewTarget());
    SmartDashboard.putNumber("RL setpoint", rearLeft.getNewTarget());
    SmartDashboard.putNumber("RR setpoint", rearRight.getNewTarget());

    SmartDashboard.putNumber("vel SPARK FL",
        frontLeft.getCurrentVelocityMetersPerSecond());
    SmartDashboard.putNumber("vel  SPARK FR",
        frontRight.getCurrentVelocityMetersPerSecond());
    SmartDashboard.putNumber("vel SPARK RL",
        rearLeft.getCurrentVelocityMetersPerSecond());
    SmartDashboard.putNumber("vel SPARK RR",
        rearRight.getCurrentVelocityMetersPerSecond());

  }

  /**
   * method for driving the robot
   * Parameters:
   * forward linear value
   * sideways linear value
   * rotation value
   * if the control is field relative or robot relative
   */
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    // update the drive inputs for use in AlignWithGyro and AlignWithTargetVision
    // control
    commandedForward = forward;
    commandedStrafe = strafe;
    commandedRotation = rotation;

    isCommandedFieldRelative = isFieldRelative;

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */
    ChassisSpeeds speeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = Swerve.kinematics.toSwerveModuleStates(speeds);

    // make sure the wheels don't try to spin faster than the maximum speed possible
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.maxSpeed);

    setModuleStates(states);

  }

  /**
   * Method to set the desired state for each swerve module
   * Uses PID and feedforward control to control the linear and rotational values
   * for the modules
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {

    frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
    frontRight.setDesiredStateClosedLoop(moduleStates[1]);
    rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
    rearRight.setDesiredStateClosedLoop(moduleStates[3]);

  }

  // returns an array of SwerveModuleState
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = {
        new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getIntegratedAngle()),
        new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getIntegratedAngle())
    };

    return states;

  }

  /**
   * Return the current position of the robot on field
   * Based on drive encoder and gyro reading
   */
  public Pose2d getPose() {

    return odometry.getPoseMeters();

  }

  // reset the current pose to a desired pose
  public void resetPose(Pose2d pose) {

    navX.reset();
    odometry.resetPosition(pose, getHeading());

  }

  // reset the measured distance driven for each module
  public void resetDriveDistances() {

    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

  }

  // //return the average distance driven for each module to get an overall
  // distance
  // // driven by the robot
  // public double getAverageDriveDistanceRadians() {

  // return
  // ((Math.abs(frontLeft.getDriveDistanceRadians())
  // + Math.abs(frontRight.getDriveDistanceRadians())
  // + Math.abs(rearLeft.getDriveDistanceRadians())
  // + Math.abs(rearRight.getDriveDistanceRadians()) / 4.0;

  // }

  // // return the average velocity for each module to get an overall velocity for
  // // the robot
  // public double getAverageDriveVelocityRadiansPerSecond() {

  // return ((Math.abs(frontLeft.getCurrentVelocityRadiansPerSecond())
  // + Math.abs(frontRight.getCurrentVelocityRadiansPerSecond())
  // + Math.abs(rearLeft.getCurrentVelocityRadiansPerSecond())
  // + Math.abs(rearRight.getCurrentVelocityRadiansPerSecond())) / 4.0);

  // }

  // get the current heading of the robot based on the gyro
  public Rotation2d getHeading() {

    return Rotation2d.fromDegrees(-navX.getYaw());

  }

  public double[] getCommandedDriveValues() {

    double[] values = { commandedForward, commandedStrafe, commandedRotation };

    return values;

  }

  public boolean getIsFieldRelative() {

    return isCommandedFieldRelative;

  }

  public void resetImu() {

    navX.reset();

  }

}