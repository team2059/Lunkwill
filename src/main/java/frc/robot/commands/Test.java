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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveBase;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class Test extends SwerveControllerCommand {
  private Limelight limeLight;
  private SwerveBase swerveBase;
 
  /** Creates a new Test. */
  // public Test(Limelight limelight, SwerveBase swervebase) {
  public Test(
    
      Supplier<Pose2d> pose, 
      SwerveDriveKinematics kinematics, 
      PIDController xController,
      PIDController yController, 
      ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates, 
      Subsystem  requirements, SwerveBase swervebase, Limelight limelight) {
        super(
           null, //trajectory, 
           pose, 
           kinematics, 
           xController,
           yController, 
           thetaController,
           outputModuleStates, 
            requirements);
    // Use addRequirements() here to declare subsystem dependencies.
        limeLight = limelight;
        swerveBase = swervebase;
      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    
    var startingPose = new Pose2d(0, 0, new Rotation2d());
    var result = limeLight.getCamera().getLatestResult();

    var bestTarget = result.getBestTarget();

    // target pose
    var endingPose = new Pose2d(bestTarget.getBestCameraToTarget().getX() - Units.inchesToMeters(7),
        bestTarget.getBestCameraToTarget().getY(),
        new Rotation2d(bestTarget.getBestCameraToTarget().getRotation().getZ() - Math.PI));

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(endingPose.getX() / 3.0, endingPose.getY() / 3.0));
    interiorWaypoints.add(new Translation2d(2.0 * endingPose.getX() / 3.0, 2.0 * endingPose.getY() / 3.0));

    TrajectoryConfig config = new TrajectoryConfig(0.75, 0.5);
    config.setReversed(false);

    var trajectory = TrajectoryGenerator.generateTrajectory(
        startingPose,
        interiorWaypoints,
        endingPose,
        config);

         

    swerveBase.resetOdometry(trajectory.getInitialPose());

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(1, 0,
        0);
    PIDController yController = new PIDController(1, 0,
        0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // return new SwerveControllerCommand(
    //     trajectory,
    //     swerveBase::getPose,
    //     Swerve.kinematics,
    //     xController,
    //     yController,
    //     thetaController,
    //     swerveBase::setModuleStates,
    //     swerveBase);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
