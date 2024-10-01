// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveBase;

public class TeleopXboxSwerveCmd extends Command {

  private final Timer timer;

  private final SwerveBase swerveSubsystem;
  private final DoubleSupplier forwardX, forwardY, rotation;
  private final SlewRateLimiter forwardXSlewRateLimiter, forwardYSlewRateLimiter, rotationSlewRateLimiter;

  /** Creates a new SwerveJoystickCommand. */
  public TeleopXboxSwerveCmd(SwerveBase swerveSubsystem, DoubleSupplier forwardX, DoubleSupplier forwardY, DoubleSupplier rotation) {
    timer = new Timer();

    this.swerveSubsystem = swerveSubsystem;
    this.forwardX = forwardX;
    this.forwardY = forwardY;
    this.rotation = rotation;

    this.forwardXSlewRateLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAcceleration);
    this.forwardYSlewRateLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAcceleration);
    this.rotationSlewRateLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAngularAcceleration);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(RobotContainer.xboxController.getPOV()) {
      case 0:
        // Up pressed
        Constants.SwerveConstants.currentSpeedFactor += 0.01;
        break;

      case 180:
        // Down pressed
        Constants.SwerveConstants.currentSpeedFactor -= 0.01;
        break;

      default:
        // Everything else (do nothing)

    }

    if (Constants.SwerveConstants.currentSpeedFactor < 0.11) {
      Constants.SwerveConstants.currentSpeedFactor = 0.11;
    } else if (Constants.SwerveConstants.currentSpeedFactor > 1.0) {
      Constants.SwerveConstants.currentSpeedFactor = 1.0;
    }

    SmartDashboard.putNumber("Speed Limit", Constants.SwerveConstants.currentSpeedFactor);

    // get joystick input as x, y, and rotation
    double xSpeed = forwardX.getAsDouble() * Constants.SwerveConstants.currentSpeedFactor;
    double ySpeed = -forwardY.getAsDouble() * Constants.SwerveConstants.currentSpeedFactor;
    double rot = rotation.getAsDouble() * Constants.SwerveConstants.currentSpeedFactor;

    SmartDashboard.putNumber("ROT", rot);

    // Apply deadband
    xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed : 0.0;
    rot = Math.abs(rot) > 0.1 ? rot : 0.0;

    // get distance from center of joystick, scale to 0-1 value, rumble xbox controller
    // z^2 = x^2 + y^2
    RobotContainer.xboxController.setRumble(
      RumbleType.kBothRumble, 
      0.7 * Math.sqrt(Math.pow(Math.abs(xSpeed), 2) + Math.pow(Math.abs(ySpeed), 2))
    );

    //RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, Math.abs(xSpeed));

    // Apply rate limits
    // double sliderLimit = -((slider.getAsDouble() - 1) / 2);
    // if (sliderLimit < 0.2) sliderLimit = 0.2;
    // xSpeed = forwardXSlewRateLimiter.calculate(xSpeed) * sliderLimit;
    // ySpeed = forwardYSlewRateLimiter.calculate(ySpeed) * sliderLimit;
    // rot = rotationSlewRateLimiter.calculate(rot) * sliderLimit;

    xSpeed = forwardXSlewRateLimiter.calculate(xSpeed);
    ySpeed = forwardYSlewRateLimiter.calculate(ySpeed);
    rot = rotationSlewRateLimiter.calculate(rot);
    
    swerveSubsystem.drive(xSpeed, ySpeed, rot * 1.5, RobotContainer.fieldRelativeStatus);

    // // Create chassisSpeeds to set to states
    // ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot * 1.5);

    // // Create the modulestates
    // SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
  
    // swerveSubsystem.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
