package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterBedIO
{
    @AutoLog
    public static class ShooterBedIOInputs
    {
        public double     bedAppliedVolts = 0.0;
        public double[]   bedCurrentAmps  = new double[] {};
        public Rotation2d bedAngle        = new Rotation2d();
    }

    public default void updateInputs(ShooterBedIOInputs inputs)
    {
    }

    public default void setVoltage(double volts)
    {
    }

    public default void setAngleOffset(Rotation2d bedAbsoluteEncoderOffset)
    {
    }
}
