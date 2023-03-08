package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
        public static final class PowerDistribution {
                public static final int pdpID = 15;
        }

        public static final class ArmConstants {

                public static final int tiltId = 16;
                public static final int extendId = 9;
                public static final int thruBoreDIO = 0;
                public static final double tiltkP = 7.5;
                public static final double tiltkD = 0.01;
                public static final double extensionkP = 0.01;

        }

        public static final class Pneumatics {
                public static final int pneumaticsHubId = 17;

        }

        public static final class Presets {

                public static final double CUBE_LIMELIGHT_OFFSET_INCHES = 2.5;
                public static final double CONE_LIMELIGHT_TAG_OFFSET_INCHES_LEFT = 23;
                public static final double CONE_LIMELIGHT_TAG_OFFSET_INCHES_RIGHT = -23;

                public static final double PICKUP_TILT = 0.35;
                public static final double PICKUP_EXTEND = 14.47;

                public static final double REST_TILT = 0.33;
                public static final double REST_EXTEND = -1;

                public static final double LOW_CUBE_ARM_TILT = 0.355;
                public static final double LOW_CUBE_ARM_EXTEND = 5;

                public static final double MID_CUBE_ARM_TILT = 0.52;
                public static final double MID_CUBE_ARM_EXTEND = 20;

                public static final double UPPER_CUBE_ARM_TILT = 0.55;
                public static final double UPPER_CUBE_ARM_EXTEND = 45;

                public static final double MID_CONE_ARM_TILT = 0.51;
                public static final double MID_CONE_ARM_EXTEND = 19;

                public static final double UPPER_CONE_ARM_TILT = 0.54;
                public static final double UPPER_CONE_ARM_EXTEND = 45;
        }

        public static final class Swerve {

                public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 0.15, 0.01);

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(18.75);
                public static final double wheelBase = Units.inchesToMeters(24.5);

                // nominal (real) divided by fudge factor
                public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1.04085);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
                public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1

                public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left
                                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
                                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
                );

                /* Swerve Profiling Values */
                public static final double maxSpeed = 4.5; // meters per second
                public static final double maxAngularVelocity = 11.5;

                public static final int frontLeftRotationMotorId = 7;
                public static final int frontLeftDriveMotorId = 8;

                public static final int frontRightRotationMotorId = 5;
                public static final int frontRightDriveMotorId = 6;

                public static final int rearLeftRotationMotorId = 3;
                public static final int rearLeftDriveMotorId = 4;

                public static final int rearRightRotationMotorId = 1;
                public static final int rearRightDriveMotorId = 2;

                public static final int frontLeftRotationEncoderId = 12;
                public static final int frontRightRotationEncoderId = 13;
                public static final int rearLeftRotationEncoderId = 14;
                public static final int rearRightRotationEncoderId = 11;

                public static final double kTeleDriveMaxSpeedMetersPerSecond = 7.5 / 4.0;
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.5;
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

                public static final double cameraToFrontEdgeDistanceMeters = Units.inchesToMeters(7);

        }

        public static final class LimelightConstants {
                public static final double xCameraOffsetInches = 2;
                public static final double yCameraOffsetInches = -5.5;
                // public static final double originToFront = Units.inchesToMeters(18.75 +
                // 14.625);
                public static final double originToFrontInches = 33;

               

        }

        public static final class AutoConstants {

                public static final double kPXController = 0.75;
                public static final double kPYController = 0.75;
                public static final double kPThetaController = 5;

                public static final double kMaxSpeedMetersPerSecond = Swerve.maxSpeed / 4;
                public static final double kMaxAngularSpeedRadiansPerSecond = //
                                Swerve.maxAngularVelocity / 10;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                                new TrapezoidProfile.Constraints(
                                                kMaxAngularSpeedRadiansPerSecond,
                                                kMaxAngularAccelerationRadiansPerSecondSquared);

        }

}
