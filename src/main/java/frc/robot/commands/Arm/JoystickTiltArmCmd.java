// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TiltArm;

public class JoystickTiltArmCmd extends CommandBase {
  private TiltArm tiltArm;
  private DoubleSupplier tiltOutput;

  /** Creates a new JoystickTiltArmCmd. */
  public JoystickTiltArmCmd(TiltArm tiltArm, DoubleSupplier tiltOutput) {
    this.tiltArm = tiltArm;
    this.tiltOutput = tiltOutput;
    addRequirements(tiltArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(tiltOutput.getAsDouble()) < 0.3) {
      tiltArm.getTiltMotor().set(0);
    } else {

      tiltArm.getTiltMotor().set(tiltOutput.getAsDouble());
    }
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
