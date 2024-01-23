package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {

  private final VisionIO _io;
  private final PhotonCamera _camera;
  private final PhotonPoseEstimator _poseEstimator;
  private final VisionIOInputsAutoLogged _inputs = new VisionIOInputsAutoLogged();
  private Pose2d _memory = new Pose2d();

  public Vision(VisionIO io) {
    _io = io;

    AprilTagFieldLayout aprilTagFieldLayout = null;

    try {
      // Load the AprilTag field layout from the resource file
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (Exception e) {
      System.out.println("Exception encountered: " + e.getMessage());
    }

    _camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);
    _poseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            _camera,
            new Transform3d(
                Constants.Vision.PARAMETER_X,
                Constants.Vision.PARAMETER_Y,
                Constants.Vision.PARAMETER_Z,
                Constants.Vision.PARAMETER_ROTATION));
    _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
  }

  @Override
  public void periodic() {
    _io.updateInputs(_inputs);
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(_memory);

    if (estimatedPose.isPresent()) {
      _memory = estimatedPose.get().estimatedPose.toPose2d();
    }
  }

  private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    _poseEstimator.setLastPose(prevEstimatedRobotPose);
    return _poseEstimator.update();
  }
}
