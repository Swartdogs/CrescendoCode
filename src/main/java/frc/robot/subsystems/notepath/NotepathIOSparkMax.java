package frc.robot.subsystems.notepath;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class NotepathIOSparkMax implements NotepathIO
{
    private CANSparkMax _notepathSparkMax;

    @SuppressWarnings("resource")
    public NotepathIOSparkMax(int notePathCanId, int followerCanID)
    {
        _notepathSparkMax = new CANSparkMax(notePathCanId, MotorType.kBrushless);
        CANSparkMax followerSparkMax = new CANSparkMax(followerCanID, MotorType.kBrushless);
        followerSparkMax.follow(_notepathSparkMax, true);
    }

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
        inputs.notepathAppliedVolts = _notepathSparkMax.getAppliedOutput() * _notepathSparkMax.getBusVoltage();
        inputs.notepathCurrentAmps = new double[]
        { _notepathSparkMax.getOutputCurrent() };
    }

    @Override
    public void setVoltage(double volts)
    {
        _notepathSparkMax.setVoltage(volts);
    }
}
