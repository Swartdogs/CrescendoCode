package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private Pose2d memory = new Pose2d();

    public Vision(VisionIO io) {
        this.io = io;
        camera = new PhotonCamera("frontCam");
        poseEstimator = new PhotonPoseEstimator(
                /* AprilTagFieldLayout */ null,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                /* Transform3d */ new Transform3d(Units.inchesToMeters(12), 0.0, Units.inchesToMeters(5.25), new Rotation3d()));
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
