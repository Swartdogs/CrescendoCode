package frc.robot.subsystems.shooter;

public interface ShooterFlywheelIO 
{
    public static class ShooterFlywheelIOInputs
    {
        public double flywheelAppliedVolts = 0.0;
        public double[] flywheelCurrentAmps = new double[]{};
    }
    
    public default void updateInputs(ShooterFlywheelIOInputs inputs){}

    public default void setVoltage(double volts){}
} 
