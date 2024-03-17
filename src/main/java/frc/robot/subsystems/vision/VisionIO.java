package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO
{
    @AutoLog
    public static class VisionIOInputs
    {
        public double   captureTimestamp    = 0.0;
        public double[] cornerX             = new double[] {};
        public double[] cornerY             = new double[] {};
        public double[] closeCornerX        = new double[] {};
        public double[] closeCornerY        = new double[] {};
        public Pose2d   pose                = new Pose2d();
        public boolean  hasPose             = false;
        public int      numProcessedTargets = 0;
        public double[] targetDistances     = new double[] {};
    }

    public default void updateInputs(VisionIOInputs inputs)
    {
    }
}
