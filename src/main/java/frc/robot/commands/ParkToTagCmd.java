// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveBase;

public class ParkToTagCmd extends CommandBase {
  SwerveBase swerveBase;
  Limelight limelight;
  boolean done = false;
  Command followPath;

  /** Creates a new ParkToTagCmd. */
  public ParkToTagCmd(SwerveBase swerveBase, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerveBase = swerveBase;
    addRequirements(swerveBase, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // robot pose
    var startingPose = new Pose2d(0, 0, new Rotation2d());

    var bestTarget = limelight.getCamera().getLatestResult().getBestTarget();

    if (bestTarget == null) {
      done = true;
      return;
    }

    // target pose
    var endingPose = new Pose2d(bestTarget.getBestCameraToTarget().getX(),
        bestTarget.getBestCameraToTarget().getY(),
        new Rotation2d(bestTarget.getBestCameraToTarget().getRotation().getZ() - Math.PI));

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(endingPose.getX() / 3.0, endingPose.getY() / 3.0));
    interiorWaypoints.add(new Translation2d(2.0 * endingPose.getX() / 3.0, 2.0 * endingPose.getY() / 3.0));

    TrajectoryConfig config = new TrajectoryConfig(0.25, 0.25);
    config.setReversed(false);

    var trajectory = TrajectoryGenerator.generateTrajectory(
        startingPose,
        interiorWaypoints,
        endingPose,
        config);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(1, 0,
        0);
    PIDController yController = new PIDController(1, 0,
        0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    followPath = new SwerveControllerCommand(
        trajectory,
        swerveBase::getPose,
        Swerve.kinematics,
        xController,
        yController,
        thetaController,
        swerveBase::setModuleStates,
        swerveBase);

    SequentialCommandGroup fullCmd = new SequentialCommandGroup(

        new InstantCommand(() -> swerveBase.resetOdometry(trajectory.getInitialPose())),
        // new InstantCommand(() -> swerveBase.resetOdometry(offsetedPose)),
        followPath,
        new InstantCommand(() -> swerveBase.stopModules()));

    fullCmd.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
