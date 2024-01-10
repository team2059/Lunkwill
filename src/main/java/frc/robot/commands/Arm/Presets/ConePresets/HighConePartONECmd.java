// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Presets.ConePresets;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.commands.Arm.ExtendToSetpointSequenceCmd;
import frc.robot.commands.Arm.PIDTiltArmCmd;

import frc.robot.subsystems.TiltArm;
import frc.robot.subsystems.ExtendArm;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighConePartONECmd extends SequentialCommandGroup {

  /** Creates a new MidCubeCmd. */
  public HighConePartONECmd(TiltArm tiltArm, ExtendArm extendArm) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PIDTiltArmCmd(tiltArm, Constants.Presets.UPPER_CONE_ARM_TILT),
        new ExtendToSetpointSequenceCmd(extendArm, Constants.Presets.UPPER_CONE_ARM_EXTEND));
  }
}
