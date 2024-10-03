// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosly.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int JoystickPort = 1;
    public static final int XboxControllerPort = 0;

    // Axes
    public static final int JoystickTranslationAxis = 1;
    public static final int JoystickStrafeAxis = 0;
    public static final int JoystickRotationAxis = 2;
    public static final int JoystickSliderAxis = 3;

    // Buttons
    public static final int JoystickRobotRelative = 12;
    public static final int JoystickLoadNote = 11;
    public static final int JoystickShootNote = 10;
  }

  public static class ShooterConstants {
    public static final int topDriveMotorId = 16;
    public static final int bottomDriveMotorId = 17;
    public static final int indexerMotorId = 15;
  }

  public static class SwerveConstants {
    public static double currentSpeedFactor = 0.5;

    // flipped because originally there was too much slipping
    public static final double trackWidth = Units.inchesToMeters(24.5);
    public static final double wheelBase = Units.inchesToMeters(18.5);

    // nominal (real) divided by fudge factor
    public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1.0);
    public static final double wheelCircumference = 2.0 * wheelDiameter * Math.PI;

    // Kinematics gets each module relative to center. X is forward/backward and Y is left/right, left is positive
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left 
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // back left
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // back right
    );

    public static final double kPTurning = 0.25;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double rotationGearRatio = (150.0 / 7.0);

    public static final double driveBaseRadius = Units.inchesToMeters(18.5);

    public static final double maxMetersPerSecond = 1;

    // Swerve Modules: CAN IDs and offsets for the CANcoders.
      // CANcoder offsets provided by Tuner X are scaled 0-1, must convert to radians
    // front left
    public static final int frontLeftDriveMotorId = 7;
    public static final int frontLeftRotationMotorId = 8;
    public static final int frontLeftCanCoderId = 12;
    public static final double frontLeftOffsetRad = 0.287842 * 2 * Math.PI;
    // front right
    public static final int frontRightDriveMotorId = 5;
    public static final int frontRightRotationMotorId = 6;
    public static final int frontRightCanCoderId = 11;
    public static final double frontRightOffsetRad = 0.617676 * 2 * Math.PI;
    // back left
    public static final int backLeftDriveMotorId = 3;
    public static final int backLeftRotationMotorId = 4;
    public static final int backLeftCanCoderId = 13;
    public static final double backLeftOffsetRad = 0.899658 * 2 * Math.PI;
    // back right
    public static final int backRightDriveMotorId = 1;
    public static final int backRightRotationMotorId = 2;
    public static final int backRightCanCoderId = 14;
    public static final double backRightOffsetRad = 0.496826 * 2 * Math.PI;
    public static final double kTeleDriveMaxAcceleration = 10;
    public static final double kTeleDriveMaxAngularAcceleration = 10;
  }
}
