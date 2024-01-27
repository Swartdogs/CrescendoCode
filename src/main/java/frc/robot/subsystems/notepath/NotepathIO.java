package frc.robot.subsystems.notepath;

public interface NotepathIO
{
    public static class NotepathInputs
    {
        public double notepathAppliedVolts = 0.0;
        public double[] notepathCurrentAmps = new double[] {};
    }

    public default void updateInputs(NotepathInputs inputs){}

    public default void setVoltage(double volts){}
}
