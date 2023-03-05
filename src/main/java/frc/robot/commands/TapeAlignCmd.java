// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveBase;

public class TapeAlignCmd extends CommandBase {
  private final Limelight limelight;
  private final SwerveBase swerveBase;

  final char direction;
  final double DRIVE_P = 0.1;
  final double DRIVE_D = 0.0;
  final double STRAFE_OFFSET = 0;
  PIDController strafeController = new PIDController(DRIVE_P, 0, DRIVE_D);
  double strafeSpeed;

  /** Creates a new AutoAlignCmd. */
  public TapeAlignCmd(Limelight limelight, SwerveBase swerveBase, char direction) {
    this.limelight = limelight;
    this.swerveBase = swerveBase;
    this.direction = direction;
    addRequirements(limelight, swerveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setTapeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = limelight.getCamera().getLatestResult();

    if (result.hasTargets()) {
      // Calculate angular turn power
      double yaw = result.getBestTarget().getYaw();
     // SmartDashboard.putNumber("pitch valueCMD", pitch);
      strafeSpeed = strafeController.calculate(yaw, 0+STRAFE_OFFSET);
     // SmartDashboard.putNumber("strafeSpeed speed", strafeSpeed);

    } else {
      strafeSpeed = 0;
    }
    swerveBase.drive(0, strafeSpeed, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setTagMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
