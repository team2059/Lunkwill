// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier forwardX, forwardY, rotation;
  private final Supplier<Boolean> robotRelative;
  private final SlewRateLimiter forwardXSlewRateLimiter, forwardYSlewRateLimiter, rotationSlewRateLimiter;

  /** Creates a new SwerveJoystickCommand. */
  public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier forwardX, DoubleSupplier forwardY, DoubleSupplier rotation, Supplier<Boolean> robotRelative) {
    this.swerveSubsystem = swerveSubsystem;
    this.forwardX = forwardX;
    this.forwardY = forwardY;
    this.rotation = rotation;
    this.robotRelative = robotRelative;

    this.forwardXSlewRateLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAcceleration);
    this.forwardYSlewRateLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAcceleration);
    this.rotationSlewRateLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAngularAcceleration);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get joystick input as x, y, and rotation
    double xSpeed = forwardX.getAsDouble();
    double ySpeed = -forwardY.getAsDouble();
    double rot = rotation.getAsDouble();

    // Apply deadband
    xSpeed = Math.abs(xSpeed) > 0.25 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.33 ? ySpeed : 0.0;
    rot = Math.abs(rot) > 0.4 ? rot : 0.0;

    // Apply rate limits
    // double sliderLimit = -((slider.getAsDouble() - 1) / 2);
    // if (sliderLimit < 0.2) sliderLimit = 0.2;
    // xSpeed = forwardXSlewRateLimiter.calculate(xSpeed) * sliderLimit;
    // ySpeed = forwardYSlewRateLimiter.calculate(ySpeed) * sliderLimit;
    // rot = rotationSlewRateLimiter.calculate(rot) * sliderLimit;

    xSpeed = forwardXSlewRateLimiter.calculate(xSpeed);
    ySpeed = forwardYSlewRateLimiter.calculate(ySpeed);
    rot = rotationSlewRateLimiter.calculate(rot);

    if (robotRelative.get() && RobotContainer.fieldRelativeStatus) {
      RobotContainer.fieldRelativeStatus = false;
    } else if (robotRelative.get() && !RobotContainer.fieldRelativeStatus) {
      RobotContainer.fieldRelativeStatus = true;
    }

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
