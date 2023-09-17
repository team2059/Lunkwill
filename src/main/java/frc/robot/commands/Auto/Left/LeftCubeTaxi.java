// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Left;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ZeroEntireArmCmd;
import frc.robot.commands.Arm.Presets.CubePresets.HighCubeCmd;
import frc.robot.commands.Arm.Presets.CubePresets.MidCubeCmd;
import frc.robot.subsystems.ExtendArm;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.TiltArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftCubeTaxi extends SequentialCommandGroup {
  /** Creates a new CenterConeTaxiAndBalanceCmd. */
  public LeftCubeTaxi(SwerveBase swerveBase, TiltArm tiltArm, ExtendArm extendArm,
      Pneumatics pneumatics) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new HighCubeCmd(tiltArm, extendArm, pneumatics).withTimeout(5), new ParallelCommandGroup(
        new ZeroEntireArmCmd(extendArm, tiltArm, pneumatics), swerveBase.followPathCmd("LeftCubeTaxi")));
  }
}
