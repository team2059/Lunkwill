// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveBase;

public class TeleopXboxSwerveCmd extends Command {

  private final Timer timer;

  private final SwerveBase swerveSubsystem;
  private final DoubleSupplier forwardX, forwardY, rotation;
  private final IntSupplier pov;
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

  public static double speedFactor = 0.5;

  /** Creates a new SwerveJoystickCommand. */
  public TeleopXboxSwerveCmd(SwerveBase swerveSubsystem, DoubleSupplier forwardX, DoubleSupplier forwardY, DoubleSupplier rotation, IntSupplier pov) {
    timer = new Timer();

    this.swerveSubsystem = swerveSubsystem;
    this.forwardX = forwardX;
    this.forwardY = forwardY;
    this.rotation = rotation;
    this.pov = pov;

    this.xLimiter = new SlewRateLimiter(SwerveConstants.maxAcceleration);
    this.yLimiter = new SlewRateLimiter(SwerveConstants.maxAcceleration);
    this.rotLimiter = new SlewRateLimiter(SwerveConstants.maxAngularAcceleration);

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

    // Account for speed factor (POV up/down buttons)
    switch(pov.getAsInt()) {
      case 0:
        // Up pressed
        speedFactor += 0.01;
        break;

      case 180:
        // Down pressed
        speedFactor -= 0.01;
        break;

      default:
        // Everything else (do nothing)

    }
    if (speedFactor < 0.11) {
      speedFactor = 0.11;
    } else if (speedFactor > 1.0) {
      speedFactor = 1.0;
    }
    SmartDashboard.putNumber("Speed Limit", speedFactor);

    /**
     * Units are given in meters per second radians per second
     * Since joysticks give output from -1 to 1, we multiply the outputs by the max
     * speed
     * Otherwise, our max speed would be 1 meter per second and 1 radian per second
     */

    // get joystick input as x, y, and rotation
    double xSpeed = forwardX.getAsDouble();
    double ySpeed = forwardY.getAsDouble();
    double rot = rotation.getAsDouble();

    // Apply deadband
    xSpeed = Math.abs(xSpeed) > 0.14 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.14 ? ySpeed : 0.0;
    rot = Math.abs(rot) > 0.1 ? rot : 0.0;

    // Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * SwerveConstants.kTeleDriveMaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveConstants.kTeleDriveMaxSpeed;
    rot = rotLimiter.calculate(rot) * SwerveConstants.kTeleDriveMaxAngularSpeed;

    xSpeed *= speedFactor;
    ySpeed *= speedFactor;
    rot *= speedFactor;
    
    swerveSubsystem.drive(
      MathUtil.applyDeadband(xSpeed, 0.1, 0.75),
      MathUtil.applyDeadband(ySpeed, 0.3, 0.75), 
      MathUtil.applyDeadband(rot, 0.3, 0.75), 
      SwerveBase.fieldRelativeStatus
    );

    // Rumble xbox controller
    // z^2 = x^2 + y^2
    // RobotContainer.xboxController.setRumble(
    //   RumbleType.kBothRumble, 
    //   0.7 * Math.sqrt(Math.pow(Math.abs(xSpeed), 2) + Math.pow(Math.abs(ySpeed), 2))
    // );
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
