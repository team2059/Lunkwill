package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANcoder canCoder;
    private final Rotation2d offset;

    private final PIDController rotationPidController;
    public SwerveModule(
        int driveMotorId,
        int rotationMotorId,
        int canCoderId,
        double canCoderOffsetRadians
    ) {
        // Instantiate motor controller objects
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);
        
        // Set brake mode as default idle mode
        driveMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.setIdleMode(IdleMode.kBrake);

        // Set encoder objects to appropriate motor's encoders
        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        /*
         * Set conversion factors for drive encoder.
         * Ensures that output is in meters (linear position) or meters/sec (linear velocity)
         */
        driveEncoder.setPositionConversionFactor(SwerveConstants.driveEncoderPositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.driveEncoderVelocityConversionFactor);

        /*
         * Set conversion factors for rotation encoder.
         * Ensures that output is in radians (angular position) or radians/sec (angular velocity)
         */
        rotationEncoder.setPositionConversionFactor(SwerveConstants.rotationEncoderPositionConversionFactor);
        rotationEncoder.setVelocityConversionFactor(SwerveConstants.rotationEncoderVelocityConversionFactor);

        // Instantiate rotation PID controller, for smoother and more accurate rotation
        rotationPidController = new PIDController(SwerveConstants.rotationkP, 0, 0);

        // Tells pidcontroller that -pi is the same as +pi, can calculate shorter path to setpoint from either sign
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);        

        // Instantiate new CANcoder and respective offset, set configuration
        canCoder = new CANcoder(canCoderId);
        offset = new Rotation2d(canCoderOffsetRadians);

        configureCanCoder();

        // Write settings to motors
        driveMotor.burnFlash();
        rotationMotor.burnFlash();
    }

    /**
     * Configure a CANcoder (absolute encoder) to operate CCW and unsigned 0-1
     */
    public void configureCanCoder() {
        // Create the new configuration
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        // Makes the range of the sensor 0-1 so that radians can be calculated
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        // Makes turning ccw positive
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // Apply cancoder configuration
        canCoder.getConfigurator().apply(canCoderConfig);
    }

    /**
     * @return current drive encoder position (distance traveled).
     * Should be in meters after proper conversion factors applied in constructor
     */
    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    /**
     * @return Rotation2d of current rotation encoder position (radians)
     */
    public Rotation2d getRotationEncoderPosition() {
        double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

        if (unsignedAngle < 0) unsignedAngle += 2 * Math.PI;

        return new Rotation2d(unsignedAngle);
    }

    /**
     * @return CANcoder object
     */
    public CANcoder getCANcoder() {
        return canCoder;
    }

    /**
     * @return current drive velocity (meters/sec assuming conversion factor is already applied)
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * @return current rotation velocity (rad/sec assuming conversion factor is already applied)
     */
    public double getRotationVelocity() {
        return rotationEncoder.getVelocity();
    }

    /**
     * @return CANSparkFlex drive motor controller object
     */
    public CANSparkMax getDriveMotor() {
        return driveMotor;
    }

    /**
     * @return CANSparkFlex rotation motor controller object
     */
    public CANSparkMax getRotationMotor() {
        return rotationMotor;
    }

    /**
     * Get actual rotation by subtracting offset from absolute reading
     * @return Rotation2d of current angle
     */
    public Rotation2d getCANcoderRad() {
        double canCoderRad = (Math.PI * 2 * canCoder.getAbsolutePosition().getValueAsDouble()) - offset.getRadians() % (2 * Math.PI);
        return new Rotation2d(canCoderRad);
    }

    /**
     * Reset encoders to their starting values. 
     * Rotation encoder starts at the CANcoder offset (zero degrees), drive encoder starts at 0 (zero meters).
     */
    public void resetEncoders() {
        rotationEncoder.setPosition(offset.getRadians());
        driveEncoder.setPosition(0.0);
    }

    /**
     * @return current SwerveModuleState of a module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getCANcoderRad());
    }

    /**
     * Adjust given angle to a range around a reference angle within [0,2π]
     * @param scopeReference reference angle in radians
     * @param newAngle angle in radians to adjust 
     * @return
     */
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % (2.0 * Math.PI);
        if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + ((2.0 * Math.PI) - lowerOffset);
        } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - ((2.0 * Math.PI) + lowerOffset);
        }
        while (newAngle < lowerBound) {
          newAngle += (2.0 * Math.PI);
        }
        while (newAngle > upperBound) {
          newAngle -= (2.0 * Math.PI);
        }
        if (newAngle - scopeReference > (Math.PI)) {
          newAngle -= (2.0 * Math.PI);
        } else if (newAngle - scopeReference < -(Math.PI)) {
          newAngle += (2.0 * Math.PI);
        }
        return newAngle;
    }

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
        double delta = (targetAngle - currentAngle.getRadians());
        if (Math.abs(delta) > (Math.PI / 2)) {
        targetSpeed = -targetSpeed;
        targetAngle = delta > Math.PI / 2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
        }
        return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
    }

    /**
     * Apply a SwerveModuleState (direct scaling for drive motor, PID for rotation)
     * @param state the target SwerveModuleState
     */
    public void setDesiredStates(SwerveModuleState state) {
        // Optimize finds the closest angle to the target
        state = optimize(state, getCANcoderRad());

        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.maxVelocity);

        // use PID for turning to avoid overshooting
        rotationMotor.set(rotationPidController.calculate(getCANcoderRad().getRadians(), state.angle.getRadians()));
    }

    /**
     * Method to set the desired state of a swerve module,
     * using PID and feedforward to control the output
     * 
     * @param desiredState SwerveModuleState object that holds desired linear and rotational setpoint
     */
    public void setDesiredStateFF(SwerveModuleState desiredState) {
        // Deadband
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Create optimized state to work with
        SwerveModuleState optimizedState = optimize(desiredState, getCANcoderRad());

        // Set outputs (PID for rotation, FF for drive)
        rotationMotor.set(rotationPidController.calculate(
            getCANcoderRad().getRadians(), // current angle
            optimizedState.angle.getRadians() // target angle
        ));
        driveMotor.setVoltage(SwerveConstants.driveFF.calculate(
            optimizedState.speedMetersPerSecond // target speed
        ));
    }

    /**
     * @return Current traveled distance in meters
     * TODO: does this need fixed?
     */
    public double getCurrentDistanceMeters() {
        return getDriveEncoderPosition() * (SwerveConstants.wheelDiameter / 2.0);
    }

    /**
     * Set both motors within a module to 0
     */
    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    @Override
    public void periodic() {
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
