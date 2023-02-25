// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Proxy;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  public ArrayList<Integer> tagIDs = new ArrayList<>();

  public final String allianceColor = DriverStation.getAlliance().toString();

  /* Controllers */
  public static final Joystick driver = new Joystick(0);
  public static final ButtonBox buttonBox = new ButtonBox(1);
  public static final ButtonBox buttonBoxTwo = new ButtonBox(2);
  public static final Joystick logitech = new Joystick(3);

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 4;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton alignWithTarget = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton autoBalance = new JoystickButton(driver, XboxController.Button.kX.value);

  // april tags
  private final JoystickButton leftTag = new JoystickButton(buttonBox, 10);;
  private final JoystickButton centerTag = new JoystickButton(buttonBoxTwo, 1);
  private final JoystickButton rightTag = new JoystickButton(buttonBoxTwo, 2);
  private final JoystickButton substationTag = new JoystickButton(buttonBoxTwo, 3);

  // middle cube nodes
  private final JoystickButton lowCube = new JoystickButton(buttonBox, 8);
  private final JoystickButton midCube = new JoystickButton(buttonBox, 5);
  private final JoystickButton highCube = new JoystickButton(buttonBox, 2);

  // private final JoystickButton tilt50 = new JoystickButton(buttonBox, 4);
  // private final JoystickButton tilt100 = new JoystickButton(buttonBox, 5);
  // private final JoystickButton extend50 = new JoystickButton(buttonBox, 6);
  // private final JoystickButton extend100 = new JoystickButton(buttonBox, 7);

  private final JoystickButton gripperSolenoidToggle = new JoystickButton(logitech, 1);;
  private final JoystickButton extenderSolenoidToggle = new JoystickButton(logitech, 2);;

  /* Subsystems */
  private final SwerveBase swerveBase = new SwerveBase();;
  private final Limelight limelight = new Limelight();
  private final Arm arm = new Arm();
  private final Pneumatics pneumatics = new Pneumatics();;
  private final PowerDistributionPanel powerDistributionPanel = new PowerDistributionPanel();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // driver = new Joystick(0);
    // buttonBox = new ButtonBox(1);

    if (allianceColor.equals("Red")) {
      tagIDs.add(8);
      tagIDs.add(7);
      tagIDs.add(6);
      tagIDs.add(4);

    } else {
      tagIDs.add(3);
      tagIDs.add(2);
      tagIDs.add(1);
      tagIDs.add(5);

    }

    System.out.println("alliance color = " + allianceColor);

    for (int i = 0; i < 4; i++) {
      System.out.println(tagIDs.get(i));
    }

    swerveBase.setDefaultCommand(new TeleopSwerve(swerveBase, () -> driver.getRawAxis(translationAxis),
        () -> driver.getRawAxis(strafeAxis), () -> driver.getRawAxis(rotationAxis),
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

    try

    {
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

    if (allianceColor.equals("Red")) {
      leftTag.onTrue(new GoToTagCmd(swerveBase, limelight, 0, 3));
      centerTag.onTrue(new GoToTagCmd(swerveBase, limelight, 0, 2));
      rightTag.onTrue(new GoToTagCmd(swerveBase, limelight, 0, 1));
      substationTag.onTrue(new GoToTagCmd(swerveBase, limelight, 0, 5));
    } else {
      leftTag.onTrue(new GoToTagCmd(swerveBase, limelight, 0, 8));
      centerTag.onTrue(new GoToTagCmd(swerveBase, limelight, 0, 7));
      rightTag.onTrue(new GoToTagCmd(swerveBase, limelight, 0, 6));
      substationTag.onTrue(new GoToTagCmd(swerveBase, limelight, 0, 4));

    }

    midCube.onTrue(new SequentialCommandGroup(new PIDTiltArmCmd(arm, Constants.Presets.MID_CUBE_ARM_TILT),
        new InstantCommand(() -> pneumatics.getExtenderSolenoid().set(kForward)),
        new InstantCommand(() -> arm.getExtensionMotor().set(-0.5)).withTimeout(0.2)

        , new PIDExtendArmCmd(arm, pneumatics,
            Constants.Presets.MID_CUBE_ARM_EXTEND),
        new InstantCommand(() -> pneumatics.toggleGripperSolenoid())));

    // tilt50.onTrue(new ProxyCommand(() -> new PIDTiltArmCmd(arm, 0.31)));
    // tilt100.onTrue(new ProxyCommand(() -> new PIDTiltArmCmd(arm, 0.55)));
    // extend50.onTrue(new SequentialCommandGroup(new InstantCommand(() ->
    // pneumatics.getExtenderSolenoid().set(kForward)),
    // new InstantCommand(() -> arm.getExtensionMotor().set(-0.5)).withTimeout(0.2),
    // new ProxyCommand(() -> new PIDExtendArmCmd(arm, pneumatics, 2.5))));
    // extend100
    // .onTrue(new SequentialCommandGroup(new InstantCommand(() ->
    // pneumatics.getExtenderSolenoid().set(kForward)),
    // new InstantCommand(() -> arm.getExtensionMotor().set(-0.5)).withTimeout(0.2),
    // new ProxyCommand(() -> new PIDExtendArmCmd(arm, pneumatics, 44))));

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
