package frc.robot.subsystems.notepath;

import org.littletonrobotics.junction.AutoLog;

public interface NotepathIO
{
    @AutoLog
    public static class NotepathInputs
    {
        public double   notepathAppliedVolts = 0.0;
        public double[] notepathCurrentAmps  = new double[] {};
    }

    public default void updateInputs(NotepathInputs inputs)
    {
    }

    public default void setVoltage(double volts)
    {
    }
}
