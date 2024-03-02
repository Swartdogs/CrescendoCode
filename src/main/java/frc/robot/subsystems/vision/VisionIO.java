package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO
{
    @AutoLog
    public static class VisionIOInputs
    {
        public double   captureTimestamp = 0.0;
        public double[] cornerX          = new double[] {};
        public double[] cornerY          = new double[] {};
        public Pose2d   pose             = new Pose2d();
        public boolean  hasPose          = false;
    }

    public default void updateInputs(VisionIOInputs inputs)
    {
    }
}
