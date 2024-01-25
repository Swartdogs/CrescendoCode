package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelIO
{
    @AutoLog
    public static class ShooterFlywheelIOInputs
    {
        public double upperFlywheelAppliedVolts = 0.0;
        public double[] upperFlywheelCurrentAmps = new double[]
        {};
        public double lowerFlywheelAppliedVolts = 0.0;
        public double[] lowerFlywheelCurrentAmps = new double[]
        {};
    }

    public default void updateInputs(ShooterFlywheelIOInputs inputs)
    {}

    public default void setVoltage(double upperVolts, double lowerVolts)
    {}
}
