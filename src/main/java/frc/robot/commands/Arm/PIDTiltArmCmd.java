// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TiltArm;

public class PIDTiltArmCmd extends Command {
  private TiltArm tiltArm;
  private double setpoint;
  double output;

  /** Creates a new PIDExtendArmCmd. */
  public PIDTiltArmCmd(TiltArm tiltArm, double setpoint) {
    this.tiltArm = tiltArm;
    this.setpoint = setpoint;
    addRequirements(tiltArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("tilt setpoint", setpoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thruBorePos = tiltArm.getThruBorePosition();
    output = tiltArm.getTiltController().calculate(thruBorePos, setpoint);
    // SmartDashboard.putNumber("thruBorePos", thruBorePos);
    SmartDashboard.putNumber("tiltOutput", output);
    SmartDashboard.putNumber("realtive tilt", tiltArm.getTiltMotor().getEncoder().getPosition());

    System.out.println("tiltOutput" + output);
    tiltArm.getTiltMotor().set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    tiltArm.getTiltMotor().set(0);
    System.out.println("ENDED TILT");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (arm.getThruBorePosition()) > (0.95 * setpoint);
    // return Math.abs(output) < 0.05;
    return (Math.abs(output) < 0.125);
  }
}
