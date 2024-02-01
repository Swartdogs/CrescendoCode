package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO
{
    @AutoLog
    public static class IntakeIOInputs
    {
        public double   appliedVolts = 0.0;
        public double[] currentAmps  = new double[] {};
    }

    public default void updateInputs(IntakeIOInputs inputs)
    {
    }

    public default void setVoltage(double volts)
    {
    }
}
