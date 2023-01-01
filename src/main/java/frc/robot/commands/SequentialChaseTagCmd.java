// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialChaseTagCmd extends SequentialCommandGroup {
  SwerveBase swerveBase;
  Limelight limelight;

  /** Creates a new SequentialChaseTagCmd. */
  public SequentialChaseTagCmd(SwerveBase swerveBase,
      Limelight limelight) {
    this.limelight = limelight;
    this.swerveBase = swerveBase;
    addRequirements(limelight, swerveBase);
    addCommands(new SelectCommand(() -> getCommand()), new InstantCommand(() -> swerveBase.stopModules()));
  }

  public Command getCommand() {
    System.out.println("INIT START");
    // robot pose
    var startingPose = new Pose2d(0, 0, new Rotation2d());

    var result = limelight.getCamera().getLatestResult();

    if (result.hasTargets() == false) {
      System.out.println("has target " + result.hasTargets());
      System.out.println("no target to make best!");

      return new InstantCommand();
    } else {
      System.out.println("has target " + result.hasTargets());
      var bestTarget = result.getBestTarget();
      double yawTheta = bestTarget.getBestCameraToTarget().getRotation().getZ();


      // figure out better solution
      swerveBase.resetOdometry(new Pose2d());

      TurnToAngleCmd turnToAngle = new TurnToAngleCmd(swerveBase, Units.radiansToDegrees(yawTheta - Math.PI));

      // x = x offset between robot center (center of the drivetrain) to camera
      // y = y offset between robot center (center of the drivetrain) to camera
      Translation2d robotCenterToCameraTranslationOffset = new Translation2d(Units.inchesToMeters(5.25),
          Units.inchesToMeters(1));

      // direction vector BEFORE robot actually turns to be parallel to april tag
      Translation2d aprilTagPositionInCameraFrame = new Translation2d(bestTarget.getBestCameraToTarget().getX(),
          bestTarget.getBestCameraToTarget().getY());

      // direction vector BEFORE robot actually turns to be parallel to april tag,
      // WITH the robot's center and camera offsetted
      Translation2d aprilTagPositionInRobotCenterFrame = robotCenterToCameraTranslationOffset
          .plus(aprilTagPositionInCameraFrame);

      // CHECK sign (angle to turn robot parallel to april tag)
      Rotation2d robotRotationYawToAprilTag = new Rotation2d(yawTheta - Math.PI);

      // rotate current direction vector relative to robot's center AFTER accoutning
      // for robot's yaw change
      Translation2d aprilTagInRotatedRobotCenterFrame = aprilTagPositionInRobotCenterFrame
          .rotateBy(robotRotationYawToAprilTag);

      // offset for robot's center to front edge
      Translation2d robotCenterToRobotFrontEdge = new Translation2d(Units.inchesToMeters(11), 0);

      // final direction vector of april tag to robot 's front edge
      Translation2d finalDirectionVector = aprilTagInRotatedRobotCenterFrame.minus(robotCenterToRobotFrontEdge);

      // final april tag pose relative to all offsets
      var endingPose = new Pose2d(finalDirectionVector, new Rotation2d(0));

      // System.out
      // .println("endingX = " + endingX + " endingY = " + endingY + " endingTheta = "
      // + endingTheta.getDegrees());

      // var endingPose = new Pose2d(0, 3, new Rotation2d(0));

      SmartDashboard.putString("ending pose", endingPose.toString());

      var interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(new Translation2d(endingPose.getX() / 3.0, endingPose.getY() / 3.0));
      interiorWaypoints.add(new Translation2d(2.0 * endingPose.getX() / 3.0, 2.0 * endingPose.getY() / 3.0));

      TrajectoryConfig config = new TrajectoryConfig(2.25, 0.75);
      config.setReversed(false);

      var trajectory = TrajectoryGenerator.generateTrajectory(
          startingPose,
          interiorWaypoints,
          endingPose,
          config);

      // swerveBase.resetOdometry(trajectory.getInitialPose());

      // 3. Define PID controllers for tracking trajectory
      PIDController xController = new PIDController(0.375, 0,
          0);
      PIDController yController = new PIDController(0.4, 0,
          0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      return turnToAngle.withTimeout(5);

      // return new SequentialCommandGroup( new SwerveControllerCommand(
      // trajectory,
      // swerveBase::getPose,
      // Swerve.kinematics,
      // xController,
      // yController,
      // thetaController,
      // swerveBase::setModuleStates,
      // swerveBase));

    }
  }
}
