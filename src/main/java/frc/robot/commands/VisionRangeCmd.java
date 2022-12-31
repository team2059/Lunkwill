// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.SwerveBase;

// public class VisionRangeCmd extends CommandBase {
//   private final Limelight limelight;
//   private final SwerveBase swerveBase;

//   final double ANGULAR_P = 0.1;
//   final double ANGULAR_D = 0.0;
//   PIDController forwardController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
//   double rotationSpeed;
//   double forwardSpeed;
//   final double GOAL_RANGE_METERS = Units.inchesToMeters(12);

//   /** Creates a new AutoAlignCmd. */
//   public VisionRangeCmd(Limelight limelight, SwerveBase swerveBase) {
//     this.limelight = limelight;
//     this.swerveBase = swerveBase;
//     addRequirements(limelight, swerveBase);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (limelight.hasTargets()) {
      

//       SmartDashboard.putNumber("target distance inches", Units.metersToInches(range));

//       forwardSpeed = forwardController.calculate(range, GOAL_RANGE_METERS);

//     } else {
//       forwardSpeed = 0;

//     }
//     swerveBase.drive(forwardSpeed, 0, 0, false);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
