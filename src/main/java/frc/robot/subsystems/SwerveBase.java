// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveBase extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  // Create 4 SwerveModule objects using given constants.
  private final SwerveModule frontLeft = new SwerveModule(
    SwerveConstants.frontLeftDriveMotorId, 
    SwerveConstants.frontLeftRotationMotorId, 
    SwerveConstants.frontLeftCanCoderId, 
    SwerveConstants.frontLeftOffsetRad);
  private final SwerveModule frontRight = new SwerveModule(
    SwerveConstants.frontRightDriveMotorId, 
    SwerveConstants.frontRightRotationMotorId, 
    SwerveConstants.frontRightCanCoderId, 
    SwerveConstants.frontRightOffsetRad);
  private final SwerveModule backLeft = new SwerveModule(
    SwerveConstants.backLeftDriveMotorId, 
    SwerveConstants.backLeftRotationMotorId, 
    SwerveConstants.backLeftCanCoderId, 
    SwerveConstants.backLeftOffsetRad);
  private final SwerveModule backRight = new SwerveModule(
    SwerveConstants.backRightDriveMotorId, 
    SwerveConstants.backRightRotationMotorId, 
    SwerveConstants.backRightCanCoderId, 
    SwerveConstants.backRightOffsetRad);

  // Create NavX object (gyro)
  private final AHRS navX;

  // Create swerve drive odometry engine, used to track robot on field
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.SwerveConstants.kinematics, new Rotation2d(), getModulePositions());

  public SwerveBase() {

    // NavX may need an extra second to start...
    navX = new AHRS(SPI.Port.kMXP);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
      } catch (Exception e) {
      }
    }).start();

    // Reset encoders upon each start
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();

    // Drive motor inversions
    // Offsets can mess with these sometimes
    frontLeft.getDriveMotor().setInverted(true);
    frontRight.getDriveMotor().setInverted(true);
    backLeft.getDriveMotor().setInverted(true);
    backRight.getDriveMotor().setInverted(false);

    // Rotation motor inversions -- all or nothing situation
    frontLeft.getRotationMotor().setInverted(true);
    frontRight.getRotationMotor().setInverted(true);
    backLeft.getRotationMotor().setInverted(true);
    backRight.getRotationMotor().setInverted(true);

    configureAutoBuilder();
  }

  /**
   * Get current robot pose
   * @return current Pose2d in meters
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Get AHRS object (navX gyro)
   * @return AHRS navX object
   */
  public AHRS getNavX() {
    return navX;
  }

  /**
   * Set odometry to specified pose, with current heading and module positions
   * @param pose the specified Pose2d
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /**
   * @return ChassisSpeeds relative to the robot
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = SwerveConstants.kinematics.toChassisSpeeds(getStates());

    return chassisSpeeds;
  }

  /**
   * Resets heading (yaw) of the navX to zero
   */
  public void zeroHeading() {
    navX.reset();
  }

  /**
   * @return Rotation2d of current robot heading
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getYaw());
  }

  /**
   * @return SwerveModulePosition[] current positions of all four modules
   * [fl, fr, bl, br]
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      new SwerveModulePosition(-frontLeft.getCurrentDistanceMeters(), frontLeft.getCANcoderRad()),
      new SwerveModulePosition(-frontRight.getCurrentDistanceMeters(), frontRight.getCANcoderRad()),
      new SwerveModulePosition(-backLeft.getCurrentDistanceMeters(), backLeft.getCANcoderRad()),
      new SwerveModulePosition(-backRight.getCurrentDistanceMeters(), backRight.getCANcoderRad())
    };

    return positions;
  }

  /**
   * Drive in robot-relative mode
   * @param chassisSpeeds target ChassisSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] newStates = Constants.SwerveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.SwerveConstants.maxVelocity);
    setModuleStates(newStates);
  }

  /**
   * Drive in field-relative mode
   * @param chassisSpeeds target ChassisSpeeds
   */
  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    discreteSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getHeading());
    SwerveModuleState[] newStates = Constants.SwerveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.SwerveConstants.maxVelocity);
    setModuleStates(newStates);
  }

  /**
   * @return SwerveModuleState[] states of all four modules (contains speed & angle of each module)
   *  [fl, fr, bl, br]
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = frontLeft.getState();
    states[1] = frontRight.getState();
    states[2] = backLeft.getState();
    states[3] = backRight.getState();

    return states;
  }

  /**
   * Set the state of all modules
   * @param desiredStates SwerveModuleStates[] to set
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // makes it never go above 5 m/s
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxVelocity);
    // Sets the speed and rotation of each module
    frontLeft.setDesiredStateFF(desiredStates[0]);
    frontRight.setDesiredStateFF(desiredStates[1]);
    backLeft.setDesiredStateFF(desiredStates[2]);
    backRight.setDesiredStateFF(desiredStates[3]);

    Logger.recordOutput("Target States", desiredStates);
  }
  
  /**
   * Drive the robot (combines driveFieldRelative and driveRobotRelative methods)
   * @param forward linear y-axis input [-1,1]
   * @param strafe linear x-axis input [-1,1]
   * @param rotation z-axis input [-1,1]
   * @param isFieldRelative boolean field relativity switch
   */
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */

    ChassisSpeeds speeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxVelocity);

    setModuleStates(states);

  }

  /**
   * Set field relativity
   * When called, it flips the current value
   */
  public void setFieldRelativity() {
    if (fieldRelativeStatus) {
      fieldRelativeStatus = false;
    } else {
      fieldRelativeStatus = true;
    }
  }

  /**
   * Method to configure AutoBuilder, used for auto routines
   */
  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
      this::getPose, // robot pose supplier
      this::resetOdometry, // method to reset odometry (will be called if auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier (must be robot relative)
      this::driveRobotRelative, // method that will drive the robot given robot relative ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(0, 0, 0), // Translation PID constants
        new PIDConstants(0, 0, 0), // Rotation PID constants
        SwerveConstants.maxVelocity, // Max module speed in m/s
        SwerveConstants.driveBaseRadius, // drive base radius in meters, distance from robot center to furthest module
        new ReplanningConfig()// Default path replanning config, see the API for options
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
  }

  /*
   * This method is automatically run approximately every 20ms
   */
  @Override
  public void periodic() {

    // Update robot position on the field
    odometry.update(getHeading(), getModulePositions());

    // Logger - for various dashboards
    Logger.recordOutput("NavX Angle", navX.getAngle());
    Logger.recordOutput("Real States", getStates());
    Logger.recordOutput("Pose", getPose());
    Logger.recordOutput("FIELD-RELATIVE?", fieldRelativeStatus);

  }
}
