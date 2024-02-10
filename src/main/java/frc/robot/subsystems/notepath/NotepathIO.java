package frc.robot.subsystems.notepath;

import org.littletonrobotics.junction.AutoLog;

public interface NotepathIO
{
    @AutoLog
    public static class NotepathInputs
    {
        public double   leaderNotepathAppliedVolts   = 0.0;
        public double[] leaderNotepathCurrentAmps    = new double[] {};
        public double   followerNotepathAppliedVolts = 0.0;
        public double[] followerNotepathCurrentAmps  = new double[] {};
        public boolean  sensorTripped                = false;
    }

    public default void updateInputs(NotepathInputs inputs)
    {
    }

    public default void setVoltage(double volts)
    {
    }
}
