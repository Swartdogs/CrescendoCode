package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelIO
{
    @AutoLog
    public static class ShooterFlywheelIOInputs
    {
        public double   upperVelocity = 0.0;
        public double   upperVolts    = 0.0;
        public double[] upperCurrent  = new double[] {};
        public double   lowerVelocity = 0.0;
        public double   lowerVolts    = 0.0;
        public double[] lowerCurrent  = new double[] {};
    }

    public default void updateInputs(ShooterFlywheelIOInputs inputs)
    {
    }

    public default void setUpperVelocity(double velocity)
    {
    }

    public default void setLowerVelocity(double velocity)
    {
    }

    public default void setUpperVolts(double volts)
    {
    }

    public default void setLowerVolts(double volts)
    {
    }
}
