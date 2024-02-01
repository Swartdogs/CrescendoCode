package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterBedIO
{
    @AutoLog
    public static class ShooterBedIOInputs
    {
        public double     bedLeaderAppliedVolts = 0.0;
        public double[]   bedLeaderCurrentAmps  = new double[] {};
        
        public double     bedFollowerAppliedVolts = 0.0;
        public double[]   bedFollowerCurrentAmps  = new double[] {};

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
