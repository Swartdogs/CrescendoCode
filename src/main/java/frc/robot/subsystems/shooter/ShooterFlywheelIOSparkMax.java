package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterFlywheelIOSparkMax implements ShooterFlywheelIO
{
    private CANSparkMax _flywheelSparkMax;

    public ShooterFlywheelIOSparkMax(int flywheelCanID)
    {
        _flywheelSparkMax = new CANSparkMax(flywheelCanID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        inputs.flywheelAppliedVolts = _flywheelSparkMax.getAppliedOutput() * _flywheelSparkMax.getBusVoltage();
        inputs.flywheelCurrentAmps = new double[]{_flywheelSparkMax.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double volts)
    {
        _flywheelSparkMax.setVoltage(volts);
    }
}
