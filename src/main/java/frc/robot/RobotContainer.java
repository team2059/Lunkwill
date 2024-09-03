// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.NoteControl;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  private static final Shooter shooter = new Shooter(ShooterConstants.indexerMotorId, ShooterConstants.driveMotorId);

  private final static XboxController xboxController = new XboxController(OperatorConstants.XboxControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    allianceChooser.addOption("RED", true);
    allianceChooser.setDefaultOption("BLUE", false);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Alliance Chooser", allianceChooser);

    // Send axes & buttons from joystick to SwerveJoystickCommand,
      // which will govern the SwerveSubsystem
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
      swerveSubsystem, 
      () -> xboxController.getLeftY(),
      () -> -xboxController.getLeftX(), 
      () -> xboxController.getRightX(),
      () -> xboxController.getXButton()
    ));
    
    shooter.setDefaultCommand(new NoteControl(
      shooter, 
      () -> xboxController.getRightTriggerAxis(), 
      () -> xboxController.getLeftTriggerAxis(),
      () -> xboxController.getBButton()
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
    new JoystickButton(xboxController, 7).onTrue(new InstantCommand(() -> swerveSubsystem.getNavX().zeroYaw()));

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
