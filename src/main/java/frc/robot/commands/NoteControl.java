// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class NoteControl extends Command {

  private final Shooter shooter;
  DoubleSupplier indexerSpeed, driveSpeed;
  BooleanSupplier inverted;

  /** Creates a new NoteControl. */
  public NoteControl(Shooter shooter, DoubleSupplier indexerSpeed, DoubleSupplier driveSpeed, BooleanSupplier inverted) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSpeed = driveSpeed;
    this.indexerSpeed = indexerSpeed;
    this.shooter = shooter;
    this.inverted = inverted;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (inverted.getAsBoolean()) {
      shooter.setBothMotorsSpeed(-0.4);
    } else {
      shooter.setDriveMotorSpeed(driveSpeed.getAsDouble());
      shooter.setIndexerMotorSpeed(indexerSpeed.getAsDouble());
    }
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
