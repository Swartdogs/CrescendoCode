package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO
{
    @AutoLog
    public static class ClimbIOInputs
    {
        public double leftExtension     = 0.0;
        public double rightExtension    = 0.0;
        public double leftVolts  = 0.0;
        public double rightVolts = 0.0;
    }

    public default void updateInputs(ClimbIOInputs inputs)
    {
    }

    public default void setLeftVolts(double volts)
    {
    }

    public default void setRightVolts(double volts)
    {
    }

    public default void setLeftOffset(double leftAbsoluteEncoderOffset)
    {
    }

    public default void setRightOffset(double rightAbsoluteEncoderOffset)
    {
    }
}
