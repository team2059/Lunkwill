// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pneumatics;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ManualExtendCmd extends CommandBase {
  private Arm arm;
  private Pneumatics pneumatics;

  double output;

  /** Creates a new PIDExtendArmCmd. */
  public ManualExtendCmd(Arm arm, Pneumatics pneumatics) {
    this.arm = arm;
    this.pneumatics = pneumatics;

    addRequirements(arm, pneumatics);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("extend setpoint", setpoint);
    // pneumatics.getExtenderSolenoid().set(kForward);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output = RobotContainer.logitech.getRawAxis(1) * 0.5;

    if (Math.abs(output) < 0.05) {
      output = 0;
      pneumatics.setExtenderState(kReverse);
    } else {
      pneumatics.setExtenderState(kForward);
      arm.getExtensionMotor().set(output);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // pneumatics.getExtenderSolenoid().set(kReverse);
    // pneumatics.toggleExtenderSolenoid();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return arm.getExtendPosition() >= 0.9 * setpoint;
    return false;
  }
}
