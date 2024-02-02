package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class ShooterFlywheelIOSparkMax implements ShooterFlywheelIO
{
    private CANSparkMax     _upperFlywheelSparkMax;
    private CANSparkMax     _lowerFlywheelSparkMax;
    private RelativeEncoder _upperFlywheelEncoder;
    private RelativeEncoder _lowerFlywheelEncoder;

    private SparkPIDController _upperPIDController;
    private SparkPIDController _lowerPIDController;

    public ShooterFlywheelIOSparkMax()
    {
        _upperFlywheelSparkMax = new CANSparkMax(Constants.CAN.SHOOTER_FLYWHEEL_UPPER, MotorType.kBrushless);
        _lowerFlywheelSparkMax = new CANSparkMax(Constants.CAN.SHOOTER_FLYWHEEL_LOWER, MotorType.kBrushless);
        _lowerFlywheelSparkMax.setInverted(true); // FIXME: Change if not correct

        _upperFlywheelEncoder = _upperFlywheelSparkMax.getEncoder();
        _lowerFlywheelEncoder = _lowerFlywheelSparkMax.getEncoder();

        _upperPIDController = _upperFlywheelSparkMax.getPIDController();
        _lowerPIDController = _lowerFlywheelSparkMax.getPIDController();

        for (var pid : new SparkPIDController[] { _upperPIDController, _lowerPIDController })
        {
            pid.setP(6e-5);
            pid.setI(0);
            pid.setD(0);
            pid.setIZone(0);
            pid.setFF(0.000015);
            pid.setOutputRange(-1, 1);
        }
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        inputs.upperFlywheelVelocity     = _upperFlywheelEncoder.getVelocity();
        inputs.upperFlywheelAppliedVolts = _upperFlywheelSparkMax.getAppliedOutput() * _upperFlywheelSparkMax.getBusVoltage();
        inputs.upperFlywheelCurrentAmps  = new double[] { _upperFlywheelSparkMax.getOutputCurrent() };

        inputs.lowerFlywheelVelocity     = _lowerFlywheelEncoder.getVelocity();
        inputs.lowerFlywheelAppliedVolts = _lowerFlywheelSparkMax.getAppliedOutput() * _lowerFlywheelSparkMax.getBusVoltage();
        inputs.lowerFlywheelCurrentAmps  = new double[] { _lowerFlywheelSparkMax.getOutputCurrent() };
    }

    @Override
    public void setUpperVelocity(double upperVelocity)
    {
        _upperPIDController.setReference(upperVelocity, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setLowerVelocity(double lowerVelocity)
    {
        _lowerPIDController.setReference(lowerVelocity, CANSparkMax.ControlType.kVelocity);
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
