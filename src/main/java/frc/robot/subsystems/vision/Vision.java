package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase
{

    private final VisionIO _io;
    private final PhotonCamera _camera;
    private final PhotonPoseEstimator _poseEstimator;
    private final VisionIOInputsAutoLogged _inputs = new VisionIOInputsAutoLogged();
    private Pose2d _memory = new Pose2d();

    public Vision(VisionIO io)
    {
        _io = io;

        AprilTagFieldLayout aprilTagFieldLayout = null;

        try
        {
            // Load the AprilTag field layout from the resource file
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            System.out.println("Loaded the AprilTag field layout from the resource file");
        }
        catch (Exception e)
        {
            System.out.println("Exception encountered: " + e.getMessage());
        }

        _camera = new PhotonCamera("frontCam");
        _poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _camera,
                new Transform3d(Units.inchesToMeters(12), 0.0, Units.inchesToMeters(5.25), new Rotation3d()));
        _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(_memory);

        if (estimatedPose.isPresent())
        {
            _memory = estimatedPose.get().estimatedPose.toPose2d();

            System.out.println("Time: " + estimatedPose.get().timestampSeconds + ", Pose: " + _memory);
        }
    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose)
    {
       _poseEstimator.setLastPose(prevEstimatedRobotPose);
        return _poseEstimator.update();
    }
}
