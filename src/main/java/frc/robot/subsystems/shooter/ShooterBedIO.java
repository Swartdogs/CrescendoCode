package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterBedIO  
{
    @AutoLog
    public static class ShooterBedIOInputs
    {
        public double positionRad = 0.0;
        public Rotation2d absolutePosition = new Rotation2d();
        public double shooterAngle = 0.0;
        public double bedAppliedVolts = 0.0;
        public double[] bedCurrentAmps = new double[]{};
    }   

    public default void updateInputs(ShooterBedIOInputs inputs){}

    public default void setVoltage(double volts){}
}
