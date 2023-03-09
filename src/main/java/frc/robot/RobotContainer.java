// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

import frc.robot.commands.Arm.JoystickExtendArmCmd;
import frc.robot.commands.Arm.JoystickTiltArmCmd;

import frc.robot.commands.Arm.PickUpElementArmPositionCmd;
import frc.robot.commands.Arm.ZeroEntireArmCmd;
import frc.robot.commands.Arm.Presets.SubstationPresetCmd;
import frc.robot.commands.Arm.Presets.ConePresets.HighConeCmd;
import frc.robot.commands.Arm.Presets.ConePresets.MidConeCmd;
import frc.robot.commands.Arm.Presets.CubePresets.HighCubeCmd;
import frc.robot.commands.Arm.Presets.CubePresets.LowCubeCmd;
import frc.robot.commands.Arm.Presets.CubePresets.MidCubeCmd;
import frc.robot.commands.Auto.TwoElementAuto;
import frc.robot.commands.Auto.Center.CenterCubeTaxi;
import frc.robot.commands.Auto.Center.CenterCubeTaxiBalance;
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

  /* XBOX CONTROLLER */
  public final XboxController controller = new XboxController(0);
  // private final int kExtendArmToZero = 4;

  // private final int kExtendArmToMax = 6;

  private final int kPickUpArmPosition = 5;
  private final int kGripperSolenoidToggle = 6;
  private final int kZeroEntireArm = 1;

  private final int kTiltAxis = 1;
  private final int kExtendAxis = 5;
  // private final JoystickButton extendArmToZero = new JoystickButton(controller,
  // kExtendArmToZero);
  private final JoystickButton pickUpArmPosition = new JoystickButton(controller, kPickUpArmPosition);
  // private final JoystickButton extendArmToMax = new JoystickButton(controller,
  // kExtendArmToMax);
  private final JoystickButton gripperSolenoidToggle = new JoystickButton(controller, kGripperSolenoidToggle);
  private final JoystickButton zeroEntireArm = new JoystickButton(controller, kZeroEntireArm);

  /* BUTTON BOX ONE */
  public final ButtonBox buttonBox = new ButtonBox(1);
  private final JoystickButton lowCube = new JoystickButton(buttonBox, 8);
  private final JoystickButton midCube = new JoystickButton(buttonBox, 5);
  private final JoystickButton highCube = new JoystickButton(buttonBox, 2);
  private final JoystickButton lowCone = new JoystickButton(buttonBox, 9);
  private final JoystickButton midCone = new JoystickButton(buttonBox, 6);
  private final JoystickButton highCone = new JoystickButton(buttonBox, 3);

  /* BUTTON BOX TWO */
  public final ButtonBox buttonBoxTwo = new ButtonBox(2);
  private final JoystickButton leftTagCone = new JoystickButton(buttonBoxTwo, 5);;
  private final JoystickButton centerTagCube = new JoystickButton(buttonBoxTwo, 6);
  private final JoystickButton rightTagCone = new JoystickButton(buttonBoxTwo, 7);

  private final JoystickButton substationTagLeft = new JoystickButton(buttonBoxTwo, 4);
  private final JoystickButton substationTagRight = new JoystickButton(buttonBoxTwo, 8);
  private final JoystickButton substationArm = new JoystickButton(buttonBoxTwo, 9);

  /* LOGITECH */
  public final Joystick logitech = new Joystick(3);
  private final int kLogitechTranslationAxis = 1;
  private final int kLogitechStrafeAxis = 0;
  private final int kLogitechRotationAxis = 2;
  private final int kLogitechSliderAxis = 3;
  private final int kZeroGyro = 5;
  private final int kFieldOriented = 1;
  private final int kInverted = 2;
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

    swerveBase.setDefaultCommand(new TeleopSwerve(swerveBase, () -> logitech.getRawAxis(kLogitechTranslationAxis),
        () -> logitech.getRawAxis(kLogitechStrafeAxis), () -> logitech.getRawAxis(kLogitechRotationAxis),
        () -> logitech.getRawAxis(kLogitechSliderAxis),
        () -> !logitech.getRawButton(kFieldOriented),
        () -> logitech.getRawButton(kInverted)));

    extendArm.setDefaultCommand(new JoystickExtendArmCmd(extendArm,
        () -> -controller.getRawAxis(kExtendAxis) * 0.5));

    tiltArm.setDefaultCommand(new JoystickTiltArmCmd(tiltArm, () -> -controller.getRawAxis(kTiltAxis) * 0.5));

    // Configure the button bindings
    configureButtonBindings();

    try

    {
      autoChooser.setDefaultOption("auto",
          new CenterCubeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics));

      // autoChooser.addOption("LeftConeTaxiBalance", new
      // LeftConeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics));

      // autoChooser.addOption("RightConeTaxiBalance",
      // new RightConeTaxiBalance(swerveBase, tiltArm, extendArm, pneumatics));

      // autoChooser.addOption("LeftConeTaxi", new LeftConeTaxi(swerveBase, tiltArm,
      // extendArm, pneumatics));

      // autoChooser.addOption("RightConeTaxi",
      // new RightConeTaxi(swerveBase, tiltArm, extendArm, pneumatics));

      // autoChooser.addOption("CenterConeAndBalanceANDTaxi",
      // new CenterConeAndBalance(swerveBase, tiltArm, extendArm, pneumatics));

      Shuffleboard.getTab("Autonomous").add(autoChooser);
    } catch (NullPointerException ex) {
      autoChooser.setDefaultOption("NULL nothing", new InstantCommand());
      DriverStation.reportError("auto choose NULL somewhere in RobotContainer.java", null);
    }
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

    zeroGyro.onTrue(new InstantCommand(() -> swerveBase.getNavX().reset()));

    centerTagCube.onTrue(new GoToTagCmd(swerveBase, limelight, 0, Constants.Presets.CUBE_LIMELIGHT_OFFSET_INCHES));
    leftTagCone
        .onTrue(
            new GoToTagCmd(swerveBase, limelight, Constants.Presets.CONE_LIMELIGHT_TAG_OFFSET_INCHES_LEFT, 0));
    rightTagCone
        .onTrue(
            new GoToTagCmd(swerveBase, limelight, Constants.Presets.CONE_LIMELIGHT_TAG_OFFSET_INCHES_RIGHT, 0));
    substationTagLeft.onTrue(
        new GoToTagCmd(swerveBase, limelight, Constants.Presets.SUBSTATION_LIMELIGHT_TAG_OFFSET_INCHES_LEFT,
            Constants.Presets.SUBSTATION_LIMELIGHT_TAG_OFFSET_INCHES_FRONT));

    substationTagRight.onTrue(
        new GoToTagCmd(swerveBase, limelight, Constants.Presets.SUBSTATION_LIMELIGHT_TAG_OFFSET_INCHES_RIGHT,
            Constants.Presets.SUBSTATION_LIMELIGHT_TAG_OFFSET_INCHES_FRONT));

    substationArm.onTrue(new SubstationPresetCmd(tiltArm, extendArm, pneumatics));

    lowCube.onTrue(new LowCubeCmd(tiltArm, extendArm, pneumatics));
    midCube.onTrue(new MidCubeCmd(tiltArm, extendArm, pneumatics));
    highCube.onTrue(new HighCubeCmd(tiltArm, extendArm, pneumatics));

    midCone.onTrue(new MidConeCmd(tiltArm, extendArm, pneumatics));
    highCone.onTrue(new HighConeCmd(tiltArm, extendArm, pneumatics));

    pickUpArmPosition.onTrue(new PickUpElementArmPositionCmd(tiltArm, extendArm, pneumatics));

    zeroEntireArm.onTrue(new ZeroEntireArmCmd(extendArm, tiltArm, pneumatics));

    gripperSolenoidToggle.toggleOnTrue(new InstantCommand(() -> pneumatics.toggleGripperSolenoid()));

    // alignWithTarget.whileTrue(new VisionAlignCmd(limelight, swerveBase));

    // autoBalance.onTrue(new ProxyCommand(() -> new AutoBalanceCmd(swerveBase)));

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

  public ExtendArm getExtendArm() {
    return extendArm;
  }
}
