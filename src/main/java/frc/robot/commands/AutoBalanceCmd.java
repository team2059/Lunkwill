// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveBase;

public class AutoBalanceCmd extends CommandBase {
  SwerveBase swerveBase;
  double roll;
  double error;
  double kDriveP;
  boolean haveIBeenTilted = false;
  boolean isCloserToCenter = false;
  double previousVelocity = 0;
  double driveSpeed = 0;
  int counter = 0;

  /** Creates a new AutoBalanceCmd. */
  public AutoBalanceCmd(SwerveBase swerveBase) {
    this.swerveBase = swerveBase;
    addRequirements(swerveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    roll = swerveBase.getNavX().getRoll();

    if (Math.abs(roll) > 10) {
      haveIBeenTilted = true;
    }

    // System.out.println("roll" + roll);
    error = 0 - roll;
    // System.out.println("error" + error);
    if (haveIBeenTilted == false) {
      driveSpeed = 0.5;
      swerveBase.drive(driveSpeed, 0, 0, true, true);

    } else {

      driveSpeed = Math.copySign(driveSpeed, error);

      double currentVelocity = swerveBase.getFrontRight().getCurrentVelocityMetersPerSecond();

      // swerveBase.drive(Math.signum(error) * 0.2, 0, 0, true);
      if (Math.signum(currentVelocity) != Math
          .signum(previousVelocity)) {
        System.out.println("counter = " + counter++);
        System.out.println(currentVelocity);
        System.out.println(previousVelocity);
        System.out.println("error =" + error);
        System.out.println("driveSpeed before = " + driveSpeed);
        driveSpeed *= 0.2;

        System.out.println("driveSpeed after = " + driveSpeed);
      }
      if (Math.abs(roll) < 6) {
        swerveBase.drive(0, 0, 0, true, true);
      } else {
        if (Math.abs(driveSpeed) < 0.2) {
          driveSpeed = Math.copySign(0.15, driveSpeed);
        }
        swerveBase.drive(driveSpeed, 0, 0, true, true);
      }
    }

    previousVelocity = swerveBase.getFrontRight().getCurrentVelocityMetersPerSecond();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE BALANCING");
    // System.out.println(swerveBase.getFrontRight().getCurrentVelocityMetersPerSecond());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (haveIBeenTilted && Math.abs(error) < 2.5);
    return (haveIBeenTilted && (Math.abs(swerveBase.getFrontRight().getCurrentVelocityMetersPerSecond()) < 5e-7));
    // return false;
  }
}
