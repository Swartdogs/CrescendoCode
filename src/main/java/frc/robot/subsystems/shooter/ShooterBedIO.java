package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterBedIO
{
    @AutoLog
    public static class ShooterBedIOInputs
    {
        public double     leaderVolts   = 0.0;
        public double     followerVolts = 0.0;
        public Rotation2d bedAngle      = new Rotation2d();
    }

    public default void updateInputs(ShooterBedIOInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
    }

    public default void setOffset(Rotation2d offset)
    {
    }
}
