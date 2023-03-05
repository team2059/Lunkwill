// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ExtendArm;
import frc.robot.subsystems.Pneumatics;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendToSetpointSequenceCmd extends SequentialCommandGroup {
  /** Creates a new ExtendSequenceCmd. */
  public ExtendToSetpointSequenceCmd(ExtendArm extendArm, Pneumatics pneumatics, double extendSetpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> pneumatics.setExtenderState(kForward)), new WaitCommand(0.1),

        new PIDExtendArmCmd(extendArm, pneumatics, extendSetpoint));
  }
}