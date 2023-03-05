// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class Limelight extends SubsystemBase {

  private PhotonCamera camera;

  public final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(34.125);
  public final double TARGET_HEIGHT_METERS = Units.inchesToMeters(25);
  public final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  /** Creates a new Limelight. */
  public Limelight() {

    camera = new PhotonCamera("hhCam");

  }

  public PhotonCamera getCamera() {
    return camera;
  }

  public BooleanSupplier hasTargetBooleanSupplier() {
    return () -> camera.getLatestResult().hasTargets();
  }

  public void takeSnapshot() {
    camera.takeInputSnapshot();
  }

  public void enableLED() {
      camera.setLED(VisionLEDMode.kOn);
  }

  public void disableLED() {
      camera.setLED(VisionLEDMode.kOff);
  }

  public void setPipeline(int pipelineIndex) {
      camera.setPipelineIndex(pipelineIndex);
  }

  public void setTagMode() {
    setPipeline(0);
    disableLED();
  }

  public void setTapeMode() {
    setPipeline(1);
    enableLED();
  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult(); // returns a PhotoPipeLine Container

    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();

    SmartDashboard.putBoolean("Has target", hasTargets);

    if (hasTargets) {

      SmartDashboard.putNumber("tag ID", result.getBestTarget().getFiducialId());
      SmartDashboard.putNumber("pose ambiguity", result.getBestTarget().getPoseAmbiguity());

      Transform3d bestCameraToTarget = result.getBestTarget().getBestCameraToTarget();

      SmartDashboard.putNumber("x (roll)",
          Units.radiansToDegrees(bestCameraToTarget.getRotation().getX()));
      SmartDashboard.putNumber("y (pitch)",
          Units.radiansToDegrees(bestCameraToTarget.getRotation().getY()));
      SmartDashboard.putNumber("z (yaw)",
          Units.radiansToDegrees(bestCameraToTarget.getRotation().getZ()));

      SmartDashboard.putNumber("x",
          bestCameraToTarget.getX());
      SmartDashboard.putNumber("y",
          bestCameraToTarget.getY());
      SmartDashboard.putNumber("z",
          bestCameraToTarget.getZ());

    }

  }
}