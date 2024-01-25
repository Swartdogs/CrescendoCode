package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterFlywheelIOSparkMax implements ShooterFlywheelIO
{
    private CANSparkMax _upperFlywheelSparkMax;
    private CANSparkMax _lowerFlywheelSparkMax;

    public ShooterFlywheelIOSparkMax(int upperFlywheelCanID, int lowerFlywheelCanID)
    {
        _upperFlywheelSparkMax = new CANSparkMax(upperFlywheelCanID, MotorType.kBrushless);
        _lowerFlywheelSparkMax = new CANSparkMax(lowerFlywheelCanID, MotorType.kBrushless);
        _lowerFlywheelSparkMax.setInverted(true); // FIXME: Change if not correct
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        inputs.upperFlywheelAppliedVolts = _upperFlywheelSparkMax.getAppliedOutput()
                        * _upperFlywheelSparkMax.getBusVoltage();
        inputs.upperFlywheelCurrentAmps = new double[]
        { _upperFlywheelSparkMax.getOutputCurrent() };

        inputs.lowerFlywheelAppliedVolts = _lowerFlywheelSparkMax.getAppliedOutput()
                        * _lowerFlywheelSparkMax.getBusVoltage();
        inputs.lowerFlywheelCurrentAmps = new double[]
        { _lowerFlywheelSparkMax.getOutputCurrent() };
    }

    @Override
    public void setUpperVelocity(double upperVelocity)
    {
        _upperFlywheelSparkMax.set(upperVelocity);
    }

    @Override
    public void setLowerVelocity(double lowerVelocity)
    {
        _lowerFlywheelSparkMax.set(lowerVelocity);
    }

    @Override
    public void setUpperVoltage(double upperVolts)
    {
        _upperFlywheelSparkMax.setVoltage(upperVolts);
    }

    @Override
    public void setLowerVoltage(double lowerVolts)
    {
        _lowerFlywheelSparkMax.setVoltage(lowerVolts);
    }
}
