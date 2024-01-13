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
import frc.robot.commands.PIDTiltArmCmd;
import frc.robot.commands.Arm.ExtendToSetpointSequenceCmd;
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
public class TwoElementAutoLeft extends SequentialCommandGroup {
  /** Creates a new CenterConeTaxiAndBalanceCmd. */
  public TwoElementAutoLeft(SwerveBase swerveBase, TiltArm tiltArm, ExtendArm extendArm,
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
        new PIDTiltArmCmd(tiltArm, Constants.Presets.UPPER_CONE_ARM_TILT),
        new ExtendToSetpointSequenceCmd(extendArm, Constants.Presets.UPPER_CONE_ARM_EXTEND),
        // new PIDTiltArmCmd(tiltArm, Constants.Presets.UPPER_CONE_ARM_TILT_AFTER),
        new InstantCommand(() -> pneumatics.toggleGripperSolenoid()), new WaitCommand(0.25),
        new PIDTiltArmCmd(tiltArm, Constants.Presets.UPPER_CONE_ARM_TILT),
        new ParallelCommandGroup(
            new ZeroEntireArmCmd(extendArm, tiltArm, pneumatics)
                .andThen(
                    new PIDTiltArmCmd(tiltArm, Constants.Presets.PICKUP_TILT),
                    new InstantCommand(() -> pneumatics.setGripperState(Value.kForward)),
                    new ExtendToSetpointSequenceCmd(extendArm, Constants.Presets.PICKUP_EXTEND)),
           // swerveBase.followPathCmd("goToElementLeft").withTimeout(5)),
        new InstantCommand(() -> pneumatics.setGripperState(Value.kReverse)),
        new WaitCommand(0.25), new ParallelCommandGroup(
           // swerveBase.followPathCmd("goBackFromElementLeft"),
            new PIDTiltArmCmd(tiltArm, Constants.Presets.UPPER_CUBE_ARM_TILT)),
        new HighCubeCmd(tiltArm, extendArm, pneumatics)));
  }
}
