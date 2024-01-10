// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ExtendArm;

import java.util.function.DoubleSupplier;

public class JoystickExtendArmCmd extends Command {

  private ExtendArm extendArm;
  private DoubleSupplier extendOutput;
  // boolean interrupted = false;

  /** Creates a new ManualExtendCmd. */
  public JoystickExtendArmCmd(ExtendArm extendArm, DoubleSupplier extendOutput) {

    this.extendArm = extendArm;
    this.extendOutput = extendOutput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extendArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (interrupted) {
    // pneumatics.toggleGripperSolenoid();
    // }
    // System.out.println(extendOutput.getAsDouble());
    if (Math.abs(extendOutput.getAsDouble()) < 0.05) {
      // extendOutput = 0;
      extendArm.getExtensionMotor().set(0);
      // pneumatics.setExtenderState(kReverse);
      extendArm.setServoAngle(Constants.ArmConstants.restServoAngle);
    } else {
      // pneumatics.setExtenderState(kForward);
      extendArm.setServoAngle(Constants.ArmConstants.extendServoAngle);
      extendArm.getExtensionMotor().set(extendOutput.getAsDouble());
    }
    // System.out.println(extendOutput);
    // if (Math.abs(extendOutput.getAsDouble()) > 0.3) {
      
    // }

    // else {
    // extendArm.getExtensionMotor().set(0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this.interrupted = interrupted;

    // pneumatics.toggleExtenderSolenoid();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
