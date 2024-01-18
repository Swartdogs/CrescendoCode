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

public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private Pose2d memory = new Pose2d();

    public Vision(VisionIO io) {
        this.io = io;

        AprilTagFieldLayout aprilTagFieldLayout = null;

        try {
            // Load the AprilTag field layout from the resource file
            aprilTagFieldLayout =
                AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            System.out.println("Loaded the AprilTag field layout from the resource file");
        } catch (Exception e) {
            System.out.println("Exception encountered: " + e.getMessage());
        }

        camera = new PhotonCamera("frontCam");
        poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                new Transform3d(Units.inchesToMeters(12), 0.0, Units.inchesToMeters(5.25), new Rotation3d()));
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(memory);

        if (estimatedPose.isPresent()) {
            memory = estimatedPose.get().estimatedPose.toPose2d();

            System.out.println("Time: " + estimatedPose.get().timestampSeconds + ", Pose: " + memory);
        }
    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setLastPose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }
}
