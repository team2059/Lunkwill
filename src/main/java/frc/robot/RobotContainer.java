// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeNoteCmd;
import frc.robot.commands.RunIndexerCmd;
import frc.robot.commands.SpinUpShooterMotorsCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  SendableChooser<Boolean> allianceChooser = new SendableChooser<>();
  SendableChooser<Command> autoChooser;

  public static boolean fieldRelativeStatus = true;

  

  // Create swerve subsystem
  private static final SwerveBase swerveSubsystem = new SwerveBase();
  
  private static final Shooter shooter = new Shooter(ShooterConstants.indexerMotorId, ShooterConstants.topDriveMotorId, ShooterConstants.bottomDriveMotorId);

  public final static XboxController xboxController = new XboxController(OperatorConstants.XboxControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand(
      "ShootAuto",
      new ParallelCommandGroup(
        new SpinUpShooterMotorsCmd(shooter, 1),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new RunIndexerCmd(shooter)
        )
      ).withTimeout(3)
    );
    autoChooser = AutoBuilder.buildAutoChooser();
    allianceChooser.addOption("RED", true);
    allianceChooser.setDefaultOption("BLUE", false);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Alliance Chooser", allianceChooser);

    // Send axes & buttons from joystick to SwerveJoystickCommand,
      // which will govern the SwerveSubsystem
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> xboxController.getLeftY(),
      () -> -xboxController.getLeftX(), 
      () -> xboxController.getRightX()
    ));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    /* WINDOW: RESET NAVX HEADING */
    new JoystickButton(xboxController, 7)
      .whileTrue(new InstantCommand(() -> swerveSubsystem.getNavX().zeroYaw()));

    /* B - INTAKE NOTE */
    new JoystickButton(xboxController, 2)
      .whileTrue(new IntakeNoteCmd(shooter, 0.2));

    /* LEFT BUMPER: SPIN SHOOTER MOTORS */
    new JoystickButton(xboxController, 5)
      .whileTrue(new SpinUpShooterMotorsCmd(shooter, 1));

    /* RIGHT BUMPER: EJECT NOTE */
    new JoystickButton(xboxController, 6)
      .whileTrue(new RunIndexerCmd(shooter));

    new JoystickButton(xboxController, 3).whileTrue(new InstantCommand(() -> swerveSubsystem.setFieldRelativity()));
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
