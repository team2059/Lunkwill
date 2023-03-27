// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveBase;

public class TurnToAngleCmd extends CommandBase {
  SwerveBase swerveBase;
  Limelight limelight;
  double angleDegrees;
  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  double odometryDegrees;
  double yaw = 0;
  PIDController turnController = new PIDController(ANGULAR_P, 0.0, ANGULAR_D);
  double rotationSpeed;
  boolean pos;
  boolean hasTarget;
  double output;

  /** Creates a new TurnToAngleCmd. */
  public TurnToAngleCmd(SwerveBase swerveBase, Limelight limelight) {
    this.swerveBase = swerveBase;
    this.limelight = limelight;
    addRequirements(swerveBase, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    turnController.enableContinuousInput(-180, 180);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = limelight.getCamera().getLatestResult();

    hasTarget = result.hasTargets();

    if (hasTarget) {
      double yawRadians = result.getBestTarget().getBestCameraToTarget().getRotation().getZ();
      yaw = Units.radiansToDegrees(yawRadians);
      if (Math.abs(yaw) >= 179.9) {
        this.cancel();
      }
      System.out.println("yaw" + yaw);
      pos = yaw > 0;
      if (pos) {
        rotationSpeed = -0.045 * (179.9 - Math.abs(yaw));
      } else {
        rotationSpeed = 0.045 * (179.9 - Math.abs(yaw));
      }
      if (Math.abs(rotationSpeed) < 0.33) {
        rotationSpeed = Math.signum(rotationSpeed) * 0.45;
      }
      // rotationSpeed = -turnController.calculate(yaw, 180);
      // if (Math.abs(rotationSpeed) < 0.5) {
      // rotationSpeed = Math.signum(rotationSpeed) * 0.6;

    } else

    {
      rotationSpeed = 0;
      this.cancel();
    }

    System.out.println("ROTATION SPEED" + rotationSpeed);
    SmartDashboard.putNumber("rotationSpeed", rotationSpeed);

    swerveBase.drive(0, 0, rotationSpeed, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(rotationSpeed) < 0.1;
    // yaw = Math.abs(yaw);
    // return yaw >= 179.;
    return hasTarget == false || Math.abs(rotationSpeed) < 0.1;
  }
}