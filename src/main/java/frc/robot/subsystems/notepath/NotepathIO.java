package frc.robot.subsystems.notepath;

import org.littletonrobotics.junction.AutoLog;

public interface NotepathIO
{
    @AutoLog
    public static class NotepathInputs
    {
        public double   leaderVolts     = 0.0;
        public double[] leaderCurrent   = new double[] {};
        public double   followerVolts   = 0.0;
        public double[] followerCurrent = new double[] {};
        public boolean  sensorTripped   = false;
    }

    public default void updateInputs(NotepathInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
    }
}
