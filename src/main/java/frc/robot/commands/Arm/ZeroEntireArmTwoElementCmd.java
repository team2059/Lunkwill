// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.PIDTiltArmCmd;
import frc.robot.subsystems.ExtendArm;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TiltArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroEntireArmTwoElementCmd extends SequentialCommandGroup {
  /** Creates a new ZeroEntireArmCmd. */
  public ZeroEntireArmTwoElementCmd(ExtendArm extendArm, TiltArm tiltArm, Pneumatics pneumatics) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> extendArm.setServoAngle(Constants.ArmConstants.extendServoAngle)),
        new PIDTiltArmCmd(tiltArm, Constants.Presets.REST_TILT + 0.06),
        new ExtendToLimitSwitch(extendArm), new PIDTiltArmCmd(tiltArm, Constants.Presets.REST_TILT),
        new InstantCommand(() -> pneumatics.setGripperState(Value.kReverse))
    // new ExtendToSetpointSequenceCmd(extendArm, Constants.Presets.REST_EXTEND),
    );
  }
}
