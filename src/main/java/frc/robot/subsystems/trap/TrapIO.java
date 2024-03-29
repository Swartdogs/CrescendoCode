package frc.robot.subsystems.trap;

import org.littletonrobotics.junction.AutoLog;

public interface TrapIO
{
    @AutoLog
    public static class TrapIOInputs
    {
        public double trapVolts = 0.0;
    }

    public default void updateInputs(TrapIOInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
    }
}
