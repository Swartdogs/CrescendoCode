package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO  
{
    @AutoLog
    public static class shooterInputs
    {
        public double shooterAngle = 0.0;
        public double turnAppliedVolts = 0.0;
        
        public double shooterAppliedVolts = 0.0;
        public double[] shooterCurrentAmps = new double[]{};
    }   

    public default void updateInputs(){}

    public default void setShooterAngle(){}

    public default void setShooterVoltage(){}
}
