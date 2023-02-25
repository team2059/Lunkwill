// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pneumatics;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class PIDExtendArmCmd extends CommandBase {
  private Arm arm;
  private Pneumatics pneumatics;
  private double setpoint;
  double output;

  /** Creates a new PIDExtendArmCmd. */
  public PIDExtendArmCmd(Arm arm, Pneumatics pneumatics, double setpoint) {
    this.arm = arm;
    this.pneumatics = pneumatics;
    this.setpoint = setpoint;
    addRequirements(arm, pneumatics);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("extend setpoint", setpoint);
    pneumatics.getExtenderSolenoid().set(kForward);
    // pneumatics.toggleExtenderSolenoid();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double position = arm.getExtendPosition();
    output = arm.getExtensionController().calculate(position, setpoint);
    System.out.println(output);
    SmartDashboard.putNumber("extendPos", position);
    SmartDashboard.putNumber("extendOutput", output);

    arm.getExtensionMotor().set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.getExtensionMotor().setIdleMode(IdleMode.kCoast);
    pneumatics.getExtenderSolenoid().set(kReverse);
    // pneumatics.toggleExtenderSolenoid();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(output) < 0.03;
    // return false;
  }
}
