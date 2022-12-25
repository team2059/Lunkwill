// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import com.pathplanner.lib.*;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);

  /* Subsystems */
  private final SwerveBase swerveBase = new SwerveBase();

  public SwerveBase getSwerveSubsytem() {
    return swerveBase;
  }

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 4;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton isFieldRelative = new JoystickButton(driver, 5);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveBase.setDefaultCommand(
        new TeleopSwerve(
            swerveBase,
            () -> -driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            !isFieldRelative.get()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> swerveBase.resetImu()));
  }

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods
  // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  //   return new SequentialCommandGroup(
  //       new InstantCommand(() -> {
  //         // Reset odometry for the first path you run during auto
  //         if (isFirstPath) {
  //           this.resetOdometry(traj.getInitialHolonomicPose());
  //         }
  //       }),
  //       new PPSwerveControllerCommand(
  //           traj,
  //           this::getPose, // Pose supplier
  //           this.kinematics, // SwerveDriveKinematics
  //           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use
  //                                       // feedforwards.
  //           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
  //           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
  //                                       // only use feedforwards.
  //           this::setModuleStates, // Module states consumer
  //           this // Requires this drive subsystem
  //       ));
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return new exampleAuto(swerveBase);
  // }
}
