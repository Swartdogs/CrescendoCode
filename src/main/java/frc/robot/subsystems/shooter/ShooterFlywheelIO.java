package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelIO
{
    @AutoLog
    public static class ShooterFlywheelIOInputs
    {
        public double   upperFlywheelVelocity     = 0.0;
        public double   upperFlywheelAppliedVolts = 0.0;
        public double[] upperFlywheelCurrentAmps  = new double[] {};
        public double   lowerFlywheelVelocity     = 0.0;
        public double   lowerFlywheelAppliedVolts = 0.0;
        public double[] lowerFlywheelCurrentAmps  = new double[] {};
    }

    public default void updateInputs(ShooterFlywheelIOInputs inputs)
    {
    }

    public default void setUpperVelocity(double upperVelocity)
    {
    }

    public default void setLowerVelocity(double lowerVelocity)
    {
    }

    public default void setUpperVoltage(double upperVolts)
    {
    }

    public default void setLowerVoltage(double lowerVolts)
    {
    }
}
