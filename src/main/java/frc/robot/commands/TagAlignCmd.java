// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveBase;

public class TagAlignCmd extends CommandBase {
  private final Limelight limelight;
  private final SwerveBase swerveBase;
  final double DRIVE_P = 0.1;
  final double DRIVE_D = 0.0;
  final double STRAFE_OFFSET = 0;
  PIDController rotationController = new PIDController(DRIVE_P, 0, DRIVE_D);
  double rotationSpeed;

  /** Creates a new AutoAlignCmd. */
  public TagAlignCmd(Limelight limelight, SwerveBase swerveBase) {
    this.limelight = limelight;
    this.swerveBase = swerveBase;
    addRequirements(limelight, swerveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // rotationController.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = limelight.getCamera().getLatestResult();

    if (result.hasTargets()) {
      // Calculate angular turn power
      double yaw = Units.radiansToDegrees(result.getBestTarget().getBestCameraToTarget().getRotation().getZ());
      System.out.println("YAW = " + yaw);
      // SmartDashboard.putNumber("pitch valueCMD", pitch);
      rotationSpeed = rotationController.calculate(yaw, 180);
      System.out.println("rotation SPeed" + rotationSpeed);
      // SmartDashboard.putNumber("strafeSpeed speed", strafeSpeed);
      if (Math.abs(rotationSpeed) < 0.025) {
        rotationSpeed = 0;
      }

    } else {
      rotationSpeed = 0;
    }
    swerveBase.drive(0, 0, rotationSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
