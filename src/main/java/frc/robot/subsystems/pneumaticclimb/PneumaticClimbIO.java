package frc.robot.subsystems.pneumaticclimb;

import org.littletonrobotics.junction.AutoLog;

public interface PneumaticClimbIO
{
    @AutoLog
    public static class PneumaticClimbIOInputs
    {
        public boolean _leftExtended;
        public boolean _rightExtended;
    }

    public default void updateInputs(PneumaticClimbIOInputs inputs)
    {

    }

    public default void setLeft(boolean state)
    {

    }

    public default void setRight(boolean state)
    {

    }
}
