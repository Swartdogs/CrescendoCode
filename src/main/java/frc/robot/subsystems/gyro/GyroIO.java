package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO
{
    @AutoLog
    public static class GyroIOInputs
    {
        public Rotation2d yawPosition          = new Rotation2d();
        public double     yawVelocityRadPerSec = 0.0;
        public Rotation2d rollPosition         = new Rotation2d();
    }

    public default void updateInputs(GyroIOInputs inputs)
    {
    }
}
