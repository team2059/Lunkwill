// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.GoToTagCmd;
import frc.robot.commands.Arm.PickUpElementArmPositionCmd;
import frc.robot.commands.Arm.ZeroEntireArmCmd;
import frc.robot.commands.Arm.ZeroEntireArmTwoElementCmd;
import frc.robot.commands.Arm.Presets.ConePresets.HighConeCmd;
import frc.robot.commands.Arm.Presets.CubePresets.HighCubeAutoCmd;
import frc.robot.commands.Arm.Presets.CubePresets.HighCubeCmd;
import frc.robot.commands.Arm.Presets.CubePresets.MidCubeCmd;
import frc.robot.subsystems.ExtendArm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.TiltArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoElementAutoRight extends SequentialCommandGroup {
  /** Creates a new CenterConeTaxiAndBalanceCmd. */
  public TwoElementAutoRight(SwerveBase swerveBase, TiltArm tiltArm, ExtendArm extendArm,
      Pneumatics pneumatics, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(new HighConeCmd(tiltArm, extendArm, pneumatics),
    // new ParallelCommandGroup(swerveBase.followPathCmd("goToElement"),
    // new SequentialCommandGroup(new WaitCommand(2.5),
    // new PickUpElementArmPositionCmd(tiltArm, extendArm, pneumatics))),
    // new InstantCommand(() -> pneumatics.setGripperState(Value.kForward)), new
    // WaitCommand(0.25),
    // new ZeroEntireArmCmd(extendArm, tiltArm, pneumatics),
    // swerveBase.followPathCmd("goBackFromElement"), new WaitCommand(1.25),
    // new GoToTagCmd(swerveBase, limelight, 0,
    // Constants.Presets.CUBE_LIMELIGHT_FRONT_OFFSET_INCHES - 2),
    // new MidCubeCmd(tiltArm, extendArm, pneumatics));

    addCommands(
        // new HighCubeAutoCmd(tiltArm, extendArm, pneumatics),
        // new ParallelCommandGroup(new ZeroEntireArmCmd(extendArm, tiltArm,
        // pneumatics),
        new ParallelCommandGroup(swerveBase.followPathCmd("goToElementRight"),
            new PickUpElementArmPositionCmd(tiltArm, extendArm, pneumatics)),
        new InstantCommand(() -> pneumatics.setGripperState(Value.kReverse)),
        new WaitCommand(0.5),
        new ZeroEntireArmTwoElementCmd(extendArm, tiltArm, pneumatics),
        swerveBase.followPathCmd("goBackFromElementRight"),
        new WaitCommand(1.5),
        new GoToTagCmd(swerveBase, limelight, Constants.Presets.CONE_LIMELIGHT_TAG_OFFSET_INCHES_RIGHT,
            Constants.Presets.CONE_LIMELIGHT_FRONT_OFFSET_INCHES),
        new HighConeCmd(tiltArm, extendArm, pneumatics));
  }
}
