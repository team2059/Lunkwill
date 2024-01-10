// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ExtendArm;

public class ExtendToLimitSwitch extends Command {
  private ExtendArm extendArm;

  double output;

  /** Creates a new PIDExtendArmCmd. */
  public ExtendToLimitSwitch(ExtendArm extendArm) {
    this.extendArm = extendArm;

    addRequirements(extendArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("extend setpoint", setpoint);
    // if (setpoint < extendArm.getExtendPosition()) {
    // extendArm.extensionController.setP(0.125);
    // } else {
    // extendArm.extensionController.setP(0.0925);
    // }
    // pneumatics.setExtenderState(kForward);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double position = extendArm.getExtendPosition();
    // output = extendArm.getExtensionController().calculate(position, setpoint);
    // System.out.println("extendOutput" + output);
    // SmartDashboard.putNumber("extendPos", position);
    // SmartDashboard.putNumber("extendOutput", output);
    // SmartDashboard.putNumber("extendOutput",
    // extendArm.getExtensionMotor().getAppliedOutput());

    extendArm.getExtensionMotor().set(-0.9);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE EXTENDING");
    extendArm.getExtensionMotor().set(0);

    extendArm.setServoAngle(Constants.ArmConstants.restServoAngle);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return arm.getExtendPosition() >= 0.9 * setpoint;
    return extendArm.getIsLimitReached() == true;
  }
}
