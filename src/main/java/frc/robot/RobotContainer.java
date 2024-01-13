// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

import frc.robot.commands.Arm.JoystickExtendArmCmd;
import frc.robot.commands.Arm.JoystickTiltArmCmd;

import frc.robot.commands.Arm.PickUpElementArmPositionCmd;
import frc.robot.commands.Arm.ZeroEntireArmCmd;
import frc.robot.commands.Arm.Presets.SubstationPresetCmd;

import frc.robot.commands.Arm.Presets.ConePresets.HighConePartONECmd;
import frc.robot.commands.Arm.Presets.ConePresets.HighConePartTWOCmd;

import frc.robot.commands.Arm.Presets.ConePresets.MidConePartONECmd;
import frc.robot.commands.Arm.Presets.ConePresets.MidConePartTWOCmd;
import frc.robot.commands.Arm.Presets.CubePresets.HighCubeCmd;
import frc.robot.commands.Arm.Presets.CubePresets.LowCubeCmd;
import frc.robot.commands.Arm.Presets.CubePresets.MidCubeCmd;
import frc.robot.commands.Auto.TwoElementAutoLeft;
import frc.robot.commands.Auto.TwoElementAutoRight;
import frc.robot.commands.Auto.Center.CenterBalance;
import frc.robot.commands.Auto.Center.CenterConeTaxi;
import frc.robot.commands.Auto.Center.CenterConeTaxiBalance;
import frc.robot.commands.Auto.Center.CenterCubeTaxi;
import frc.robot.commands.Auto.Center.CenterCubeTaxiBalance;
import frc.robot.commands.Auto.Left.LeftConeTaxi;
import frc.robot.commands.Auto.Left.LeftConeTaxiBalance;
import frc.robot.commands.Auto.Left.LeftCubeTaxi;
import frc.robot.commands.Auto.Left.LeftCubeTaxiBalance;
import frc.robot.commands.Auto.Right.RightConeTaxi;
import frc.robot.commands.Auto.Right.RightConeTaxiBalance;
import frc.robot.commands.Auto.Right.RightCubeTaxi;
import frc.robot.commands.Auto.Right.RightCubeTaxiBalance;
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

  /* XBOX CONTROLLER */
  public final XboxController controller = new XboxController(0);
  // private final int kExtendArmToZero = 4;

  // private final int kExtendArmToMax = 6;

  private final int kPickUpArmPosition = 4;
  private final int kGripperSolenoidToggle = 6;
  private final int kZeroEntireArm = 1;
  private final int kSubstationArm = 4;
  private final int kSubstationGripAndTiltUp = 3;

  private final int kTiltAxis = 1;
  private final int kExtendAxis = 5;
  // private final JoystickButton extendArmToZero = new JoystickButton(controller,
  // kExtendArmToZero);
  private final JoystickButton pickUpArmPosition = new JoystickButton(controller, kPickUpArmPosition);
  // private final JoystickButton extendArmToMax = new JoystickButton(controller,
  // kExtendArmToMax);
  private final JoystickButton gripperSolenoidToggle = new JoystickButton(controller, kGripperSolenoidToggle);
  private final JoystickButton zeroEntireArm = new JoystickButton(controller, kZeroEntireArm);
  private final JoystickButton substationArm = new JoystickButton(controller, kSubstationArm);
  private final JoystickButton substationGripAndTiltUp = new JoystickButton(controller, kSubstationGripAndTiltUp);

  /* BUTTON BOX ONE */
  public final ButtonBox buttonBox = new ButtonBox(1);
  private final JoystickButton lowCube = new JoystickButton(buttonBox, 8);
  private final JoystickButton midCube = new JoystickButton(buttonBox, 5);
  private final JoystickButton highCube = new JoystickButton(buttonBox, 2);
  private final JoystickButton lowCone = new JoystickButton(buttonBox, 9);

  private final JoystickButton midConePartONE = new JoystickButton(buttonBox, 4);
  private final JoystickButton midConePartTWO = new JoystickButton(buttonBox, 6);
  private final JoystickButton highConePartONE = new JoystickButton(buttonBox, 1);
  private final JoystickButton highConePartTWO = new JoystickButton(buttonBox, 3);

  /* BUTTON BOX TWO */
  public final ButtonBox buttonBoxTwo = new ButtonBox(2);
  private final JoystickButton leftTagCone = new JoystickButton(buttonBoxTwo, 5);;
  private final JoystickButton centerTagCube = new JoystickButton(buttonBoxTwo, 6);
  private final JoystickButton rightTagCone = new JoystickButton(buttonBoxTwo, 7);

  private final JoystickButton substationTagLeft = new JoystickButton(buttonBoxTwo, 4);
  private final JoystickButton substationTagRight = new JoystickButton(buttonBoxTwo, 8);

  private final JoystickButton cubeLED = new JoystickButton(buttonBoxTwo, 11);
  private final JoystickButton coneLED = new JoystickButton(buttonBoxTwo, 12);

  /* LOGITECH */
  public final Joystick logitech = new Joystick(3);
  private final int kLogitechTranslationAxis = 1;
  private final int kLogitechStrafeAxis = 0;
  private final int kLogitechRotationAxis = 2;
  private final int kLogitechSliderAxis = 3;
  private final int kZeroGyro = 5;
  private final int kFieldOriented = 6;
  private final int kInverted = 1;
  private final int kStrafeOnly = 2;
  private final int kSlowEverything = 3;

  private final JoystickButton zeroGyro = new JoystickButton(logitech, kZeroGyro);

  /* Driver Buttons */

  // private final JoystickButton alignWithTarget = new JoystickButton(driver,
  // XboxController.Button.kRightBumper.value);
  // private final JoystickButton autoBalance = new JoystickButton(driver,
  // XboxController.Button.kX.value);

  /* Subsystems */
  private final SwerveBase swerveBase = new SwerveBase();

  private final Limelight limelight = new Limelight();
  private final TiltArm tiltArm = new TiltArm();
  private final Pneumatics pneumatics = new Pneumatics();

  private final ExtendArm extendArm = new ExtendArm();
  private final LED led = new LED();
  // private final PowerDistributionPanel powerDistributionPanel = new
  // PowerDistributionPanel();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /* Commands */
  // InstantCommand toggleExtenderSolenoidCmd = new InstantCommand(() ->
  // pneumatics.toggleExtenderSolenoid());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    swerveBase.setDefaultCommand(new TeleopSwerve(swerveBase, () -> logitech.getRawAxis(kLogitechTranslationAxis),
        () -> logitech.getRawAxis(kLogitechStrafeAxis), () -> logitech.getRawAxis(kLogitechRotationAxis),
        () -> logitech.getRawAxis(kLogitechSliderAxis),
        () -> !logitech.getRawButton(kFieldOriented),
        () -> logitech.getRawButton(kInverted), () -> logitech.getRawButton(kStrafeOnly),
        () -> logitech.getRawButton(kSlowEverything)));

    extendArm.setDefaultCommand(new JoystickExtendArmCmd(extendArm,
        () -> -controller.getRawAxis(kExtendAxis) * 0.5));

    tiltArm.setDefaultCommand(new JoystickTiltArmCmd(tiltArm, () -> -controller.getRawAxis(kTiltAxis) * 0.5));

    // Configure the button bindings
    configureButtonBindings();

    // autoChooser.addOption("test", AutoBuilder.buildAuto("straightAuto"));
    // getSwerveBase().configureAutoBuilder();
    NamedCommands.registerCommand("arm", new PIDTiltArmCmd(tiltArm, 0.65));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // try

    // {
    // // autoChooser.setDefaultOption("auto",
    // // new CenterCubeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics));

    // // autoChooser.setDefaultOption("nothing",
    // // new InstantCommand());

    // autoChooser.setDefaultOption("CenterConeTaxi", "CenterConeTaxi");
    // autoChooser.addOption("CenterConeTaxiBalance", "CenterConeTaxiBalance");
    // autoChooser.addOption("CenterCubeTaxi", "CenterCubeTaxi");
    // autoChooser.addOption("CenterCubeTaxiBalance",
    // "CenterCubeTaxiBalance");
    // autoChooser.addOption("CenterBalance",
    // "CenterBalance");

    // autoChooser.addOption("LeftConeTaxi", "LeftConeTaxi");
    // autoChooser.addOption("LeftConeTaxiBalance", "LeftConeTaxiBalance");
    // autoChooser.addOption("LeftCubeTaxiBalance", "LeftCubeTaxiBalance");
    // autoChooser.addOption("LeftCubeTaxi", "LeftCubeTaxi");

    // autoChooser.addOption("RightCubeTaxi", "RightCubeTaxi");
    // autoChooser.addOption("RightCubeTaxiBalance",
    // "RightCubeTaxiBalance");
    // autoChooser.addOption("RightConeTaxiBalance",
    // "RightConeTaxiBalance");
    // autoChooser.addOption("RightConeTaxi", "RightConeTaxi");
    // autoChooser.addOption("TwoElementAutoRight", "TwoElementAutoRight");
    // autoChooser.addOption("TwoElementAutoLeft", "TwoElementAutoLeft");

    // Shuffleboard.getTab("Autonomous").add(autoChooser);
    // } catch (NullPointerException ex) {
    // autoChooser.setDefaultOption("NULL nothing", "nothing");
    // DriverStation.reportError("auto choose NULL somewhere in
    // RobotContainer.java", null);
    // }
  }

  public Pneumatics getPneumatics() {
    return pneumatics;
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

    // new JoystickButton(logitech, 4).onTrue(new AutoBalanceCmd(swerveBase, -1));

    zeroGyro.onTrue(new InstantCommand(() -> swerveBase.getNavX().reset()));

    centerTagCube
        .onTrue(new GoToTagCmd(swerveBase, limelight, 0, Constants.Presets.CUBE_LIMELIGHT_FRONT_OFFSET_INCHES));

    leftTagCone
        .onTrue(
            new GoToTagCmd(swerveBase, limelight, Constants.Presets.CONE_LIMELIGHT_TAG_OFFSET_INCHES_LEFT,
                Constants.Presets.CONE_LIMELIGHT_FRONT_OFFSET_INCHES));

    rightTagCone
        .onTrue(
            new GoToTagCmd(swerveBase, limelight, Constants.Presets.CONE_LIMELIGHT_TAG_OFFSET_INCHES_RIGHT,
                Constants.Presets.CONE_LIMELIGHT_FRONT_OFFSET_INCHES));

    substationTagLeft.onTrue(
        new GoToTagCmd(swerveBase, limelight, Constants.Presets.SUBSTATION_LIMELIGHT_TAG_OFFSET_INCHES_LEFT,
            Constants.Presets.SUBSTATION_LIMELIGHT_TAG_OFFSET_INCHES_FRONT));

    substationTagRight.onTrue(
        new GoToTagCmd(swerveBase, limelight, Constants.Presets.SUBSTATION_LIMELIGHT_TAG_OFFSET_INCHES_RIGHT,
            Constants.Presets.SUBSTATION_LIMELIGHT_TAG_OFFSET_INCHES_FRONT));

    substationArm.onTrue(new SubstationPresetCmd(tiltArm, extendArm, pneumatics));
    substationGripAndTiltUp.onTrue(new SubstationGripAndTiltUpCmd(tiltArm, extendArm, pneumatics));

    new JoystickButton(controller, 2).whileTrue(new TurnToAngleCmd(swerveBase, limelight));

    lowCube.onTrue(new LowCubeCmd(tiltArm, extendArm, pneumatics));
    midCube.onTrue(new MidCubeCmd(tiltArm, extendArm, pneumatics));
    highCube.onTrue(new HighCubeCmd(tiltArm, extendArm, pneumatics));

    // low cone and low cube should be relatively same
    lowCone.onTrue(new LowCubeCmd(tiltArm, extendArm, pneumatics));
    midConePartONE.onTrue(new MidConePartONECmd(tiltArm, extendArm));
    midConePartTWO.onTrue(new MidConePartTWOCmd(tiltArm, extendArm, pneumatics));
    highConePartONE.onTrue(new HighConePartONECmd(tiltArm, extendArm));
    highConePartTWO.onTrue(new HighConePartTWOCmd(tiltArm, extendArm, pneumatics));

    pickUpArmPosition.onTrue(new PickUpElementArmPositionCmd(tiltArm, extendArm, pneumatics));

    zeroEntireArm.onTrue(new ZeroEntireArmCmd(extendArm, tiltArm, pneumatics));

    gripperSolenoidToggle.toggleOnTrue(new InstantCommand(() -> pneumatics.toggleGripperSolenoid()));

    cubeLED.onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> led.setCube()),
        new WaitCommand(5),
        new InstantCommand(() -> led.setOrange())));

    coneLED.onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> led.setCone()),
        new WaitCommand(5),
        new InstantCommand(() -> led.setOrange())));

    // alignWithTarget.whileTrue(new VisionAlignCmd(limelight, swerveBase));

    // autoBalance.onTrue(new ProxyCommand(() -> new AutoBalanceCmd(swerveBase)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // try {
    // if (autoChooser.getSelected().equals("CenterConeTaxi")) {
    // return new CenterConeTaxi(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("CenterConeTaxiBalance")) {
    // return new CenterConeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("CenterCubeTaxi")) {
    // return new CenterCubeTaxi(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("CenterCubeTaxiBalance")) {
    // return new CenterCubeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("LeftConeTaxi")) {
    // return new LeftConeTaxi(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("LeftConeTaxiBalance")) {
    // return new LeftConeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("CenterBalance")) {
    // return new CenterBalance(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("LeftCubeTaxiBalance")) {
    // return new LeftCubeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("LeftCubeTaxi")) {
    // return new LeftCubeTaxi(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("RightCubeTaxi")) {
    // return new RightCubeTaxi(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("RightCubeTaxiBalance")) {
    // return new RightCubeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("RightConeTaxiBalance")) {
    // return new RightConeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("RightConeTaxi")) {
    // return new RightConeTaxi(swerveBase, tiltArm, extendArm, pneumatics);
    // }
    // if (autoChooser.getSelected().equals("TwoElementAutoLeft")) {
    // return new TwoElementAutoLeft(swerveBase, tiltArm, extendArm, pneumatics,
    // limelight);
    // }
    // if (autoChooser.getSelected().equals("TwoElementAutoRight")) {
    // return new TwoElementAutoRight(swerveBase, tiltArm, extendArm, pneumatics,
    // limelight);
    // }
    // return new InstantCommand();

    // } catch (NullPointerException ex) {
    // DriverStation.reportError("auto choose NULL somewhere in getAutonomousCommand
    // in RobotContainer.java", null);
    // return new InstantCommand();
    // }
  }

  public ExtendArm getExtendArm() {
    return extendArm;
  }

  public SwerveBase getSwerveBase() {
    return swerveBase;
  }

}
