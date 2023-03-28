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
  final double ANGULAR_P = 0.15;
  final double ANGULAR_D = 0.001;
  double yaw = 0;
  PIDController turnController = new PIDController(ANGULAR_P, 0.0, ANGULAR_D);
  double rotationSpeed;
  boolean hasTarget;
  double measurement;
  double setpoint;
  int counter = 0;

  /** Creates a new TurnToAngleCmd. */
  public TurnToAngleCmd(SwerveBase swerveBase, Limelight limelight) {
    this.swerveBase = swerveBase;
    this.limelight = limelight;
    addRequirements(swerveBase, limelight);
    turnController.enableContinuousInput(-180, 180);
    // turnController.setTolerance(0.5, 2.5);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = limelight.getCamera().getLatestResult();

    hasTarget = result.hasTargets();

    if (hasTarget == false || result.getBestTarget().getPoseAmbiguity() >= 0.2) {
      this.cancel();
    } else {
      double yawRadians = result.getBestTarget().getBestCameraToTarget().getRotation().getZ();
      yaw = Units.radiansToDegrees(yawRadians);
      setpoint = yaw - 180;
      swerveBase.getNavX().reset();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    measurement = swerveBase.getHeading().getDegrees();
    // SmartDashboard.putNumber("measurement", measurement);

    rotationSpeed = turnController.calculate(measurement, setpoint);

    System.out.println(rotationSpeed);
    // SmartDashboard.putNumber("rotationSpeed", rotationSpeed);

    swerveBase.drive(0, 0, rotationSpeed, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(rotationSpeed) < 0.1;
  }
}