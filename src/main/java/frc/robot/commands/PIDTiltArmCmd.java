// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PIDTiltArmCmd extends CommandBase {
  private Arm arm;
  private double setpoint;

  /** Creates a new PIDExtendArmCmd. */
  public PIDTiltArmCmd(Arm arm, double setpoint) {
    this.arm = arm;
    this.setpoint = setpoint;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thruBorePos = arm.getThruBorePosition();
    double output = arm.getTiltController().calculate(thruBorePos, setpoint);
    SmartDashboard.putNumber("thruBorePos", thruBorePos);
    SmartDashboard.putNumber("tiltOutput", output);
    arm.getTiltMotor().set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.getTiltMotor().setIdleMode(IdleMode.kBrake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getThruBorePosition()) > (0.95 * setpoint);
  }
}
