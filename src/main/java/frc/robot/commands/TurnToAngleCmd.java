// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveBase;

public class TurnToAngleCmd extends CommandBase {
  SwerveBase swerveBase;
  double angleDegrees;
  double ANGULAR_P;
  final double ANGULAR_D = 0.0001;
  PIDController turnController;

  double rotationSpeed;

  /** Creates a new TurnToAngleCmd. */
  public TurnToAngleCmd(SwerveBase swerveBase, double angleDegrees) {
    this.swerveBase = swerveBase;
    this.angleDegrees = angleDegrees;
    addRequirements(swerveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (angleDegrees == 90) {
      ANGULAR_P = 0.1;
    } else if (angleDegrees == 180) {
      ANGULAR_P = 0.066;
    } else if (angleDegrees == 270) {
      ANGULAR_P = 0.025;
    } else {
      ANGULAR_P = 0;
    }
    turnController = new PIDController(ANGULAR_P, 0.0, ANGULAR_D);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rotationSpeed = turnController.calculate(swerveBase.getHeading().getDegrees(), angleDegrees);
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
