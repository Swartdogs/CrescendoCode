package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO
{
    @AutoLog
    public static class IntakeIOInputs
    {
        public double   motorVolts = 0.0;
        public double[] motorCurrent  = new double[] {};
    }

    public default void updateInputs(IntakeIOInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
    }
}
