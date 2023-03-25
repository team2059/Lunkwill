// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
    swerveBase.resetOdometry(new Pose2d());
    turnController.enableContinuousInput(-360, 360);
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = limelight.getCamera().getLatestResult();

    if (result.hasTargets()) {

      yaw = Units.radiansToDegrees(result.getBestTarget().getBestCameraToTarget().getRotation().getZ());
      pos = yaw > 0;
      swerveBase.resetOdometry(new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(yaw))));

    }

    System.out.println("odom " + swerveBase.getOdometry().toString());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Calculate angular turn power
    odometryDegrees = Math.abs(swerveBase.getOdometry().getPoseMeters().getRotation().getDegrees());

    rotationSpeed = turnController.calculate(odometryDegrees,
        180);
    if (Math.abs(rotationSpeed) <= 0.5) {
      rotationSpeed = Math.signum(rotationSpeed) * 0.6;
    }
    if(pos){
      rotationSpeed = -rotationSpeed;
    }
    System.out.println("ROTATION SPEED " + rotationSpeed);

    swerveBase.drive(0, 0, rotationSpeed, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    odometryDegrees = Math.abs(odometryDegrees);
    return (odometryDegrees >= 179 && odometryDegrees < 181.5);
  }
}