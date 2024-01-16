package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;


public class Vision extends SubsystemBase {

  private static final double circleFitPrecision = 0.01;
  private static final int minTargetCount = 3; // For calculating odometry
  private static final double extraLatencySecs = 0.06; // Approximate camera + network latency

  private static final boolean alwaysIdleOn = true; // Always light the LEDs during teleop
  private static final double targetGraceSecs = 0.5;
  private static final double blinkPeriodSecs = 3.0;
  private static final double blinkLengthSecs = 0.5;

  private static final double vizMaxNoLog = 0.25; // How long to wait with no vision data before
                                                  // clearing log visualization
  private static final double vizFinalWidth = 1080.0;
  private static final double vizFinalHeight = 1920.0;
  private static final double vizOriginX = 540.0;
  private static final double vizOriginY = 1536.0;
  private static final double vizHeightMeters = 12.0;

  // FOV constants
  private static final double vpw =
      2.0 * Math.tan(VisionConstants.fovHorizontal.getRadians() / 2.0);
  private static final double vph =
      2.0 * Math.tan(VisionConstants.fovVertical.getRadians() / 2.0);

  private final VisionIO io;
  private final VisionIOInputs inputs = new VisionIOInputs();

  private double lastCaptureTimestamp = 0.0;
  private Supplier<VisionLedMode> modeSupplier;
  private Supplier<Boolean> climbModeSupplier;
  private RobotState robotState;

  private double lastTranslationsTimestamp = 0.0;
  private List<Translation2d> lastTranslations = new ArrayList<>();

  private int pipeline = 0;
  private boolean ledsOn = false;
  private boolean forceLeds = false;
  private boolean autoEnabled = false;
  private Timer targetGraceTimer = new Timer();

  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    this.io = io;
    targetGraceTimer.start();
  }

  public void setSuppliers(Supplier<VisionLedMode> modeSupplier,
      Supplier<Boolean> climbModeSupplier) {
    this.modeSupplier = modeSupplier;
    this.climbModeSupplier = climbModeSupplier;
  }

  public void setRobotState(RobotState robotState) {
    this.robotState = robotState;
  }

  /** Use to enable LEDs continuously while override is "Auto" */
  public void setForceLeds(boolean on) {
    forceLeds = on;
  }

  /** Sets the current pipeline number. */
  public void setPipeline(int pipeline) {
    this.pipeline = pipeline;
  }

  public void setAutoEnabled(boolean enabled) {
    autoEnabled = enabled;
  }

  public boolean getSimpleTargetValid() {
    return inputs.simpleValid;
  }

  public double getSimpleTargetAngle() {
    return inputs.simpleAngle;
  }
    
@Override
  public void robotPeriodic() {

    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(memory);

    // Check if any AprilTags have been detected
    if (estimatedPose.isPresent()) {
      memory = estimatedPose.get().estimatedPose.toPose2d();

      Logger.recordOutput("Position", memory);

      System.out.println("Time: " + estimatedPose.get().timestampSeconds + ", Pose: " + memory);
    }
  }
}
