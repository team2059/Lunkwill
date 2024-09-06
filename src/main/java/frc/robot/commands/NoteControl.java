// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class NoteControl extends Command {

  private final Shooter shooter;
  BooleanSupplier loadNote, indexerOn, driveOn;

  /** Creates a new NoteControl. */
  public NoteControl(Shooter shooter, BooleanSupplier indexerOn, BooleanSupplier driveOn, BooleanSupplier loadNote) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveOn = driveOn;
    this.indexerOn = indexerOn;
    this.shooter = shooter;
    this.loadNote = loadNote;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (loadNote.getAsBoolean()) {
      /* LOAD NOTE */
      shooter.setBothMotorsSpeed(-0.3);
    } else {
      if (driveOn.getAsBoolean()) {
        /* LEFT BUMPER PRESSED */
        shooter.setDriveMotorSpeed(1);
      } else {
        /* LEFT BUMPER DEPRESSED */
        shooter.setDriveMotorSpeed(0);
      }
      if (indexerOn.getAsBoolean()) {
        /* RIGHT BUMPER PRESSED */
        shooter.setIndexerMotorSpeed(1);
      } else {
        /* RIGHT BUMPER DEPRESSED */
        shooter.setIndexerMotorSpeed(0);
      }
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
