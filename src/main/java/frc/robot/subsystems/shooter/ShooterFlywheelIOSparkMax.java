package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterFlywheelIOSparkMax implements ShooterFlywheelIO
{
    private CANSparkMax _flywheelSparkMax;

    public ShooterFlywheelIOSparkMax(int flywheelCanID, int followerFlywheelCanID)
    {
        _flywheelSparkMax = new CANSparkMax(flywheelCanID, MotorType.kBrushless);

        CANSparkMax _followerFlywheelSparkMax = new CANSparkMax(followerFlywheelCanID, MotorType.kBrushless);
        _followerFlywheelSparkMax.follow(_flywheelSparkMax, true);
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        inputs.flywheelAppliedVolts = _flywheelSparkMax.getAppliedOutput() * _flywheelSparkMax.getBusVoltage();
        inputs.flywheelCurrentAmps = new double[]
        { _flywheelSparkMax.getOutputCurrent() };
    }

    @Override
    public void setVoltage(double volts)
    {
        _flywheelSparkMax.setVoltage(volts);
    }
}
