// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Presets.CubePresets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.ExtendToSetpointSequenceCmd;
import frc.robot.commands.Arm.PIDTiltArmCmd;
import frc.robot.commands.Arm.ZeroEntireArmCmd;
import frc.robot.subsystems.TiltArm;
import frc.robot.subsystems.ExtendArm;
import frc.robot.subsystems.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowCubeCmd extends SequentialCommandGroup {

  /** Creates a new MidCubeCmd. */
  public LowCubeCmd(TiltArm tiltArm, ExtendArm extendArm, Pneumatics pneumatics) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PIDTiltArmCmd(tiltArm, Constants.Presets.LOW_CUBE_ARM_TILT),
        new ExtendToSetpointSequenceCmd(extendArm, pneumatics, Constants.Presets.LOW_CUBE_ARM_EXTEND),
        new InstantCommand(() -> pneumatics.toggleGripperSolenoid()),new WaitCommand(0.5),new ZeroEntireArmCmd(extendArm, tiltArm, pneumatics));
  }
}
