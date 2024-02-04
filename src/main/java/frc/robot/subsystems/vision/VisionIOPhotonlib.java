// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** Vision hardware implementation for PhotonVision. */
public class VisionIOPhotonlib implements VisionIO
{
    private final PhotonCamera  _camera           = new PhotonCamera(Constants.Vision.CAMERA_NAME);
    private double              _captureTimestamp = 0.0;
    private double[]            _cornerX          = new double[] {};
    private double[]            _cornerY          = new double[] {};
    private PhotonPoseEstimator _poseEstimator;
    private Pose2d              _lastPose         = new Pose2d();
    private boolean             _hasPose          = false;

    public VisionIOPhotonlib()
    {
        AprilTagFieldLayout aprilTagFieldLayout = null;

        try
        {
            // Load the AprilTag field layout from the resource file
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }
        catch (Exception e)
        {
            System.out.println("Exception encountered: " + e.getMessage());
        }

        _poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _camera, new Transform3d(Constants.Vision.CAMERA_X, Constants.Vision.CAMERA_Y, Constants.Vision.CAMERA_Z, Constants.Vision.CAMERA_ROTATION)
        );
        _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);

        NetworkTableInstance.getDefault().addListener(NetworkTableInstance.getDefault().getEntry("/photonvision/" + Constants.Vision.CAMERA_NAME + "/latencyMillis"), EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event ->
        {
            PhotonPipelineResult result    = _camera.getLatestResult();
            double               timestamp = Logger.getRealTimestamp() - (result.getLatencyMillis() / 1000.0);

            List<Double> cornerXList = new ArrayList<>();
            List<Double> cornerYList = new ArrayList<>();

            for (PhotonTrackedTarget target : result.getTargets())
            {
                for (TargetCorner corner : target.getDetectedCorners())
                {
                    cornerXList.add(corner.x);
                    cornerYList.add(corner.y);
                }
            }

            Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(_lastPose, result);
            Pose2d                       newPose       = new Pose2d();
            boolean                      hasPose       = false;

            if (estimatedPose.isPresent())
            {
                newPose = estimatedPose.get().estimatedPose.toPose2d();
                hasPose = true;
            }

            synchronized (VisionIOPhotonlib.this)
            {
                _captureTimestamp = timestamp;
                _cornerX          = cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
                _cornerY          = cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
                _lastPose         = newPose;
                _hasPose          = hasPose;
            }
        });
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs)
    {
        inputs.captureTimestamp = _captureTimestamp;
        inputs.cornerX          = _cornerX;
        inputs.cornerY          = _cornerY;
        inputs.pose             = _lastPose;
        inputs.hasPose          = _hasPose;
    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult result)
    {
        _poseEstimator.setLastPose(prevEstimatedRobotPose);
        return _poseEstimator.update(result);
    }
}
