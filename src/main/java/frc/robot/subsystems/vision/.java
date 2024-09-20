// package frc.robot.subsystems.vision;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.networktables.NetworkTableEvent;
// import edu.wpi.first.networktables.NetworkTableInstance;

// import frc.robot.Constants;
// import frc.robot.subsystems.drive.Drive;

// import java.util.ArrayList;
// import java.util.EnumSet;
// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import org.photonvision.targeting.TargetCorner;

// public class VisionIOPhotonlib implements VisionIO
// {
// private final PhotonCamera _camera = new
// PhotonCamera(Constants.Vision.PHOTON_CAMERA_NAME);
// private final PhotonPoseEstimator _poseEstimator;
// private double _captureTimestamp = 0.0;
// private double[] _cornerX = new double[] {};
// private double[] _cornerY = new double[] {};
// private double[] _closeCornerX = new double[] {};
// private double[] _closeCornerY = new double[] {};
// private Pose2d _pose = new Pose2d();
// private boolean _hasPose = false;
// private double[] _distances = new double[] {};
// private int[] _ids = new int[] {};
// private double[] _yaws = new double[] {};
// private int _numProcessedTargets;

// public VisionIOPhotonlib(Drive drive)
// {
// AprilTagFieldLayout aprilTagFieldLayout = null;

// try
// {
// // Load the AprilTag field layout from the resource file
// aprilTagFieldLayout =
// AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
// }
// catch (Exception e)
// {
// System.out.println("Exception encountered: " + e.getMessage());
// }

// _poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
// PoseStrategy.CLOSEST_TO_LAST_POSE, _camera,
// Constants.Vision.CAMERA_TRANSFORM);
// //
// _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);

// NetworkTableInstance.getDefault().addListener(NetworkTableInstance.getDefault().getEntry("/photonvision/"
// + Constants.Vision.PHOTON_CAMERA_NAME + "/latencyMillis"),
// EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event ->
// {
// PhotonPipelineResult result = _camera.getLatestResult();
// double timestamp = result.getTimestampSeconds();
// List<Double> cornerXList = new ArrayList<>();
// List<Double> cornerYList = new ArrayList<>();
// List<Double> closeCornerXList = new ArrayList<>();
// List<Double> closeCornerYList = new ArrayList<>();
// List<PhotonTrackedTarget> processedTargets = new ArrayList<>();
// List<Double> distances = new ArrayList<>();
// List<Integer> ids = new ArrayList<>();
// List<Double> yaws = new ArrayList<>();

// for (PhotonTrackedTarget target : result.getTargets())
// {
// ids.add(target.getFiducialId());
// yaws.add(target.getYaw());

// for (TargetCorner corner : target.getDetectedCorners())
// {
// cornerXList.add(corner.x);
// cornerYList.add(corner.y);
// }

// var transform = target.getBestCameraToTarget();
// var distance = Math.hypot(transform.getX(), transform.getY());
// distances.add(distance);

// if (distance <= Constants.Vision.MAX_DETECTION_RANGE)
// {
// processedTargets.add(target);

// for (TargetCorner corner : target.getDetectedCorners())
// {
// closeCornerXList.add(corner.x);
// closeCornerYList.add(corner.y);
// }
// }
// }

// PhotonPipelineResult processedResult = new
// PhotonPipelineResult(result.getLatencyMillis(), processedTargets);
// processedResult.setTimestampSeconds(timestamp);

// Optional<EstimatedRobotPose> estimatedPose =
// getEstimatedGlobalPose(drive.getPose(), processedResult);
// Pose2d pose = new Pose2d();
// boolean hasPose = false;

// if (estimatedPose.isPresent())
// {
// pose = estimatedPose.get().estimatedPose.toPose2d();
// hasPose = true;
// }

// synchronized (VisionIOPhotonlib.this)
// {
// _captureTimestamp = timestamp;
// _cornerX = cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
// _cornerY = cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
// _closeCornerX =
// cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
// _closeCornerY =
// cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
// _distances = distances.stream().mapToDouble(Double::doubleValue).toArray();
// _pose = pose;
// _numProcessedTargets = processedTargets.size();
// _hasPose = hasPose;
// _ids = ids.stream().mapToInt(Integer::intValue).toArray();
// _yaws = yaws.stream().mapToDouble(Double::doubleValue).toArray();
// }
// });
// }

// @Override
// public synchronized void updateInputs(VisionIOInputs inputs)
// {
// inputs.captureTimestamp = _captureTimestamp;
// inputs.cornerX = _cornerX;
// inputs.cornerY = _cornerY;
// inputs.closeCornerX = _closeCornerX;
// inputs.closeCornerY = _closeCornerY;
// inputs.numProcessedTargets = _numProcessedTargets;
// inputs.pose = _pose;
// inputs.targetDistances = _distances;
// inputs.hasPose = _hasPose;
// inputs.targetIds = _ids;
// inputs.targetYaws = _yaws;
// }

// private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d
// prevEstimatedRobotPose, PhotonPipelineResult result)
// {
// _poseEstimator.setLastPose(prevEstimatedRobotPose);
// return _poseEstimator.update(result);
// }
// }
