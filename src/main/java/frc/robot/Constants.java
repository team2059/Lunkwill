package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final class Swerve {
        public static final double teleopTurnRateDegPerSec = 360.0; // Rate the robot will spin with full rotation
                                                                    // command
        public static final double stickDeadband = 0.1;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.254, 0.137);

        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19.5);
        public static final double wheelBase = Units.inchesToMeters(16.5);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
);
        /* Swerve Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionVelocityFactor = ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;
        public static final double angleConversionFactor = 360.0 / 12.8;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kCoast;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        public static final int frontLeftRotationMotorId = 5;
        public static final int frontLeftDriveMotorId = 6;

        public static final int frontRightRotationMotorId = 7;
        public static final int frontRightDriveMotorId = 8;

        public static final int rearLeftRotationMotorId = 3;
        public static final int rearLeftDriveMotorId = 4;

        public static final int rearRightRotationMotorId = 1;
        public static final int rearRightDriveMotorId = 2;

        public static final int frontLeftRotationEncoderId = 11;
        public static final int frontRightRotationEncoderId = 12;
        public static final int rearLeftRotationEncoderId = 10;
        public static final int rearRightRotationEncoderId = 9;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 7.5 / 4.0;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.5;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        /* Module Specific Constants */
        /* Front Left Module - Module */
        public static final class Mod0 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final double angleOffset = 285.117;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9;
            public static final double angleOffset = 232.207;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final double angleOffset = 276.504;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final double angleOffset = 11.514;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
