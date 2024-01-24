package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelIO
{
    @AutoLog
    public static class ShooterFlywheelIOInputs
    {
        public double flywheelAppliedVolts = 0.0;
        public double[] flywheelCurrentAmps = new double[]
        {};
    }

    public default void updateInputs(ShooterFlywheelIOInputs inputs)
    {}

    public default void setVoltage(double volts)
    {}
}
