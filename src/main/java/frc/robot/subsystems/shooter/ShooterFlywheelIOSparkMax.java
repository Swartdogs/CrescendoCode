package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterFlywheelIOSparkMax implements ShooterFlywheelIO
{
    private CANSparkMax _upperFlywheelSparkMax;
    private CANSparkMax _lowerFlywheelSparkMax;

    public ShooterFlywheelIOSparkMax(int flywheelCanID, int followerFlywheelCanID)
    {
        _upperFlywheelSparkMax = new CANSparkMax(flywheelCanID, MotorType.kBrushless);
        _lowerFlywheelSparkMax = new CANSparkMax(followerFlywheelCanID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        inputs.upperFlywheelAppliedVolts = _upperFlywheelSparkMax.getAppliedOutput() * _upperFlywheelSparkMax.getBusVoltage();
        inputs.upperFlywheelCurrentAmps = new double[]
        { _upperFlywheelSparkMax.getOutputCurrent()};

        inputs.lowerFlywheelAppliedVolts = _lowerFlywheelSparkMax.getAppliedOutput() * _lowerFlywheelSparkMax.getBusVoltage();
        inputs.lowerFlywheelCurrentAmps = new double[]
        { _lowerFlywheelSparkMax.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double upperVolts, double lowerVolts)
    {
        _upperFlywheelSparkMax.setVoltage(upperVolts);
        _lowerFlywheelSparkMax.setVoltage(lowerVolts);
    }
}
