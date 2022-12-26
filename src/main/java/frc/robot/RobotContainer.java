// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.opencv.core.Mat;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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
    zeroGyro.whenPressed(new InstantCommand(() -> swerveBase.getNavX().reset()));
  }

  public Trajectory jsonToTrajectory(String filename, boolean resetOdometry) {

    // Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return trajectory;
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
      System.out.println("Unable to read from file " + filename);
    }
    return null;

    // Command ramseteCommand = new RamseteCommand(
    // trajectory,
    // RobotContainer.getDriveTrainSubsystem()::getPose,
    // new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    // new SimpleMotorFeedforward(
    // DriveConstants.ksVolts,
    // DriveConstants.kvVoltSecondsPerMeter,
    // DriveConstants.kaVoltSecondsSquaredPerMeter),
    // DriveConstants.kDriveKinematics,
    // RobotContainer.getDriveTrainSubsystem()::getWheelSpeeds,
    // new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // // RamseteCommand passes volts to the callback
    // RobotContainer.getDriveTrainSubsystem()::tankDriveVolts,
    // RobotContainer.getDriveTrainSubsystem());

    // Run path following command, then stop at the end.
    // If told to reset odometry, reset odometry before running path.
    // if (resetOdometry) {
    // return new SequentialCommandGroup(
    // new InstantCommand(() -> RobotContainer.getDriveTrainSubsystem()
    // .resetOdometry(trajectory.getInitialPose())),
    // ramseteCommand);
    // } else {
    // return ramseteCommand;
    // }

  }

  public PathPlannerTrajectory getPathPlannerTrajectory(String pathName, double maxVelocity, double maxAcceleration) {
    PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration);
    PathPlannerTrajectory examplPathPlannerTrajectory = PathPlanner.loadPath(pathName, constraints, false);
    return examplPathPlannerTrajectory;
  }

  public Command getPathPlannerCommand(PathPlannerTrajectory trajectory) {

    PIDController xController = new PIDController(3, 0, 0);
    PIDController yController = new PIDController(3, 0, 0);
    PIDController thetaController = new PIDController(
        0.01, 0.0, 0.001);// AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        trajectory,
        swerveBase::getPose,
        Swerve.kinematics,
        xController,
        yController,
        thetaController,
        swerveBase::setModuleStates,
        swerveBase);

    return command;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    // AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // .setKinematics(Swerve.kinematics);

    // 2. Generate trajectory
    // Trajectory trajectory = jsonToTrajectory(
    // "pathplanner/generatedJSON/straight.wpilib.json",
    // true);

    // 3. Define PID controllers for tracking trajectory
    // PIDController xController = new PIDController(AutoConstants.kPXController, 0,
    // 0);
    // PIDController yController = new PIDController(AutoConstants.kPYController, 0,
    // 0);
    // ProfiledPIDController thetaController = new ProfiledPIDController(
    // AutoConstants.kPThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    // SwerveControllerCommand swerveControllerCommand = new
    // SwerveControllerCommand(
    // trajectory,
    // swerveBase::getPose,
    // Swerve.kinematics,
    // xController,
    // yController,
    // thetaController,
    // swerveBase::setModuleStates,
    // swerveBase);

    // PPSwerveControllerCommand ppSwerveControllerCommand = new
    // PPSwerveControllerCommand(
    // trajectory,
    // swerveBase::getPose, // Pose supplier
    // Swerve.kinematics, // SwerveDriveKinematics
    // xController, // X controller. Tune these values for your robot. Leaving them
    // 0 will only use
    // // feedforwards.
    // yController, // Y controller (usually the same values as X controller)
    // thetaController, // Rotation controller. Tune these values for your robot.
    // Leaving them 0 will
    // // only use feedforwards.
    // swerveBase::setModuleStates, // Module states consumer
    // swerveBase // Requires this drive subsystem
    // );
    PathPlannerTrajectory trajectory = getPathPlannerTrajectory("str8", 1, 2);
    System.out.println("degrees pose trajectory = " + trajectory.getInitialPose().getRotation().getDegrees());
    Command ppCommand = getPathPlannerCommand(trajectory);

    // Pose2d offsetPose = trajectory.getInitialPose();
    // offsetPose= offsetPose.plus(new Transform2d(new Translation2d(0,0),new
    // Rotation2d(Math.PI)));
    // Pose2d offsetedPose = new Pose2d(trajectory.getInitialPose().getX(),
    // trajectory.getInitialPose().getY(),
    // trajectory.getInitialPose().getRotation().rotateBy(new Rotation2d(Math.PI)));
    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(

        new InstantCommand(() -> swerveBase.resetOdometry(trajectory.getInitialPose())),
        // new InstantCommand(() -> swerveBase.resetOdometry(offsetedPose)),
        ppCommand,
        new InstantCommand(() -> swerveBase.stopModules()));

  }
}
