package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public AHRS gyro;

  // commanded values from the joysticks and field relative value to use in
  // AlignWithTargetVision and AlignWithGyro
  public double commandedForward = 0;
  public double commandedStrafe = 0;
  public double commandedRotation = 0;
  public boolean isCommandedFieldRelative = false;

  public Swerve() {
    gyro = new AHRS(SPI.Port.kMXP);

    // wait 1 second for navX to calibrate
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroGyro();
      } catch (Exception e) {
      }

    }).start();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };
  }

  // public void drive(
  // Translation2d translation, double rotation, boolean fieldRelative, boolean
  // isOpenLoop) {
  // SwerveModuleState[] swerveModuleStates =
  // Constants.Swerve.swerveKinematics.toSwerveModuleStates(
  // fieldRelative
  // ? ChassisSpeeds.fromFieldRelativeSpeeds(
  // translation.getX(), translation.getY(), rotation, getYaw())
  // : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
  // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
  // Constants.Swerve.maxSpeed);

  // for (SwerveModule mod : mSwerveMods) {
  // mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
  // }
  // }

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
            forward, strafe, rotation, getYaw())
        : new ChassisSpeeds(forward, strafe, rotation);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);

    // make sure the wheels don't try to spin faster than the maximum speed possible
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);

    setModuleStates(states);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(pose, getYaw());
  }

  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(-gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getStates());
    SmartDashboard.putNumber("navX yaw", getYaw().getDegrees());

    // mSwerveMods[0].getDriveMotor().set(0.1);
    // mSwerveMods[1].getDriveMotor().set(0.1);
    // mSwerveMods[2].getDriveMotor().set(0.1);
    // mSwerveMods[3].getDriveMotor().set(0.1);

    // mSwerveMods[0].getAngleMotor().set(0.50);
    // mSwerveMods[1].getAngleMotor().set(0.50);
    // mSwerveMods[2].getAngleMotor().set(0.50);
    // mSwerveMods[3].getAngleMotor().set(0.50);

    // mSwerveMods[0].getDriveMotor().set(-0.05);
    // mSwerveMods[1].getDriveMotor().set(-0.05);
    // mSwerveMods[2].getDriveMotor().set(-0.05);
    // mSwerveMods[3].getDriveMotor().set(-0.05);

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated angle", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
