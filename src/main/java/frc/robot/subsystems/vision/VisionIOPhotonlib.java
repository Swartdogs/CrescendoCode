// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** Vision hardware implementation for PhotonVision. */
public class VisionIOPhotonlib implements VisionIO {
  private static final String cameraName = "frontCam";
  private final PhotonCamera camera = new PhotonCamera(cameraName);

  private double captureTimestamp = 0.0;
  private double[] cornerX = new double[] {};
  private double[] cornerY = new double[] {};

  public VisionIOPhotonlib() {
    NetworkTableInstance.getDefault()
      .addListener(
        NetworkTableInstance.getDefault().getEntry("/photonvision/" + cameraName + "/latencyMillis"),
        EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
        event -> {
          PhotonPipelineResult result = camera.getLatestResult();
          double timestamp =
              Logger.getRealTimestamp() - (result.getLatencyMillis() / 1000.0);

          List<Double> cornerXList = new ArrayList<>();
          List<Double> cornerYList = new ArrayList<>();
          for (PhotonTrackedTarget target : result.getTargets()) {
            for (TargetCorner corner : target.getDetectedCorners()) {
              cornerXList.add(corner.x);
              cornerYList.add(corner.y);
            }
          }

          synchronized (VisionIOPhotonlib.this) {
            captureTimestamp = timestamp;
            cornerX = cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
            cornerY = cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
          }
        }
      );
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    inputs.captureTimestamp = captureTimestamp;
    inputs.cornerX = cornerX;
    inputs.cornerY = cornerY;
  }

  @Override
  public void setLeds(boolean enabled) {
    camera.setLED(enabled ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }
}
