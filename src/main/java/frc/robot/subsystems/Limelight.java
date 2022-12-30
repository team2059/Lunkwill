// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Limelight extends SubsystemBase {

  PhotonCamera camera;

  public PhotonCamera getCamera() {
    return camera;
  }

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(27.5);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(38);

  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(90);

  /** Creates a new Limelight. */
  public Limelight() {

    camera = new PhotonCamera("hhCam");

  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    PhotonPipelineResult result = camera.getLatestResult(); // returns a PhotoPipeLine Container

    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();

    SmartDashboard.putBoolean("Has target", hasTargets);

  }

}