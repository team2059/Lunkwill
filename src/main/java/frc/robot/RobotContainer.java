// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

// import com.pathplanner.lib.*;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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
  public static final Joystick driver = new Joystick(0);;
  public static final ButtonBox buttonBox = new ButtonBox(1);

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 4;

  /* Driver Buttons */
  private final JoystickButton zeroGyro;
  private final JoystickButton alignWithTarget;
  private final JoystickButton autoBalance;
  private final JoystickButton leftAlignTag;
  private final JoystickButton centerAlignTag;
  private final JoystickButton rightAlignTag;
  private final JoystickButton tilt50;
  private final JoystickButton tilt100;
  private final JoystickButton extend50;
  private final JoystickButton extend100;
  private final JoystickButton gripperSolenoidToggle;
  private final JoystickButton extenderSolenoidToggle;

  /* Subsystems */
  private final SwerveBase swerveBase;
  private final Limelight limelight;
  private final Arm arm;
  private final Pneumatics pneumatics;
  private final PowerDistributionPanel powerDistributionPanel;

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // driver = new Joystick(0);
    // buttonBox = new ButtonBox(1);

    zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    alignWithTarget = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    autoBalance = new JoystickButton(driver, XboxController.Button.kX.value);
    leftAlignTag = new JoystickButton(buttonBox, 1);
    centerAlignTag = new JoystickButton(buttonBox, 2);
    rightAlignTag = new JoystickButton(buttonBox, 3);

    tilt50 = new JoystickButton(buttonBox, 4);
    tilt100 = new JoystickButton(buttonBox, 5);
    extend50 = new JoystickButton(buttonBox, 6);
    extend100 = new JoystickButton(buttonBox, 7);
    gripperSolenoidToggle = new JoystickButton(buttonBox, 8);
    extenderSolenoidToggle = new JoystickButton(buttonBox, 9);

    swerveBase = new SwerveBase();
    arm = new Arm();
    limelight = new Limelight();
    pneumatics = new Pneumatics();
    powerDistributionPanel = new PowerDistributionPanel();

    swerveBase.setDefaultCommand(
        new TeleopSwerve(
            swerveBase,
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            () -> !driver.getRawButton(XboxController.Button.kLeftBumper.value)));

    // swerveBase.setDefaultCommand(
    // new TeleopSwerve(
    // swerveBase,
    // () -> 0,
    // () -> 0,
    // () -> 0,
    // () -> !driver.getRawButton(XboxController.Button.kLeftBumper.value)));

    // Configure the button bindings
    configureButtonBindings();

    try {
      autoChooser.setDefaultOption("simple",
          new SequentialCommandGroup(swerveBase.followPathCmd("simple"), new AutoBalanceCmd(swerveBase)));

      autoChooser.addOption("complex",
          new SequentialCommandGroup(swerveBase.followPathCmd("complex"), new AutoBalanceCmd(swerveBase)));

      Shuffleboard.getTab("Autonomous").add(autoChooser);
    } catch (NullPointerException ex) {
      autoChooser.setDefaultOption("NULL nothing", new InstantCommand());
      DriverStation.reportError("auto choose NULL somewhere in RobotContainer.java", null);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), anxd then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */

    zeroGyro.onTrue(new InstantCommand(() -> swerveBase.getNavX().reset()));

    leftAlignTag.onTrue(new GoToTagCmd(swerveBase, limelight, -18));
    centerAlignTag.onTrue(new GoToTagCmd(swerveBase, limelight, 0));
    rightAlignTag.onTrue(new GoToTagCmd(swerveBase, limelight, 18));

    tilt50.onTrue(new ProxyCommand(() -> new PIDTiltArmCmd(arm, 0.09)));
    tilt100.onTrue(new ProxyCommand(() -> new PIDTiltArmCmd(arm, 0.18)));
    extend50.onTrue(new ProxyCommand(() -> new PIDExtendArmCmd(arm, -110)));
    extend100.onTrue(new ProxyCommand(() -> new PIDExtendArmCmd(arm, -75)));

    gripperSolenoidToggle
        .toggleOnTrue(new InstantCommand(() -> pneumatics.toggleGripperSolenoid()));

    extenderSolenoidToggle.toggleOnTrue(new InstantCommand(() -> pneumatics.toggleExtenderSolenoid()));

    alignWithTarget.whileTrue(new VisionAlignCmd(limelight, swerveBase));

    autoBalance.onTrue(new ProxyCommand(() -> new AutoBalanceCmd(swerveBase)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    try {
      return autoChooser.getSelected();
    } catch (NullPointerException ex) {
      DriverStation.reportError("auto choose NULL somewhere in getAutonomousCommand in RobotContainer.java", null);
      return new InstantCommand();
    }
  }
}
