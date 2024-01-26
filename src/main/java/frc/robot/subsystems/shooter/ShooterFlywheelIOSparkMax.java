package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterFlywheelIOSparkMax implements ShooterFlywheelIO
{
    private CANSparkMax _upperFlywheelSparkMax;
    private CANSparkMax _lowerFlywheelSparkMax;

    private RelativeEncoder _upperFlywheelEncoder;
    private RelativeEncoder _lowerFlywheelEncoder;

    public ShooterFlywheelIOSparkMax(int upperFlywheelCanID, int lowerFlywheelCanID)
    {
        _upperFlywheelSparkMax = new CANSparkMax(upperFlywheelCanID, MotorType.kBrushless);
        _lowerFlywheelSparkMax = new CANSparkMax(lowerFlywheelCanID, MotorType.kBrushless);
        _lowerFlywheelSparkMax.setInverted(true); // FIXME: Change if not correct

        _upperFlywheelEncoder = _upperFlywheelSparkMax.getEncoder();
        _lowerFlywheelEncoder = _lowerFlywheelSparkMax.getEncoder();
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        inputs.upperFlywheelVelocity = _upperFlywheelEncoder.getVelocity();
        inputs.upperFlywheelAppliedVolts = _upperFlywheelSparkMax.getAppliedOutput()
                        * _upperFlywheelSparkMax.getBusVoltage();
        inputs.upperFlywheelCurrentAmps = new double[]
        { _upperFlywheelSparkMax.getOutputCurrent() };

        inputs.lowerFlywheelVelocity = _lowerFlywheelEncoder.getVelocity();
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
