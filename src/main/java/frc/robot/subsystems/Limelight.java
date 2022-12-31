// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class Limelight extends SubsystemBase {

  private PhotonCamera camera;
  // Constants such as camera and target height stored. Change per robot and goal!
  public final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(11);
  public final double TARGET_HEIGHT_METERS = Units.inchesToMeters(25);

  // Angle between horizontal and the camera.
  public final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // public ArrayList<double> =

  public double getRangeToTargetMeters() {

    double range = 0;

    try {
      range = PhotonUtils.calculateDistanceToTargetMeters(
          CAMERA_HEIGHT_METERS,
          TARGET_HEIGHT_METERS,
          CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(getResult().getBestTarget().getPitch()));
    } catch (NullPointerException ex) {
      range = -1;
    }

    return range;
  }

  public int getTagId() {
    if (hasTargets()) {
      return getResult().getBestTarget().getFiducialId();
    }
    return -1;
  }

  public PhotonPipelineResult getResult() {

    return camera.getLatestResult();
  }

  public boolean hasTargets() {

    return getResult().hasTargets();
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  public double getTargetYawDegrees() {
    double degrees = 0;
    try {
      degrees = camera.getLatestResult()
          .getBestTarget().getYaw();

      return degrees;
    } catch (NullPointerException ex) {
      degrees = -999;

    }
    return degrees;

  }

  /** Creates a new Limelight. */
  public Limelight() {

    camera = new PhotonCamera("hhCam");

  }

  @Override
  public void periodic() {
    // // Query the latest result from PhotonVision
    // result = camera.getLatestResult(); // returns a PhotoPipeLine Container

    // // Check if the latest result has any targets.
    // hasTargets = result.hasTargets();

    SmartDashboard.putBoolean("Has target", hasTargets());

    if (hasTargets() == true) {
      SmartDashboard.putNumber("range inches", Units.metersToInches(getRangeToTargetMeters()));
      SmartDashboard.putNumber("target Yaw", getTargetYawDegrees());
      SmartDashboard.putNumber("tag ID", getTagId());
    }

  }

}