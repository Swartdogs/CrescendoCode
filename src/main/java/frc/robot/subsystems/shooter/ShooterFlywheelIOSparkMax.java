package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class ShooterFlywheelIOSparkMax implements ShooterFlywheelIO
{
    private final CANSparkMax        _upperMotor;
    private final CANSparkMax        _lowerMotor;
    private final RelativeEncoder    _upperEncoder;
    private final RelativeEncoder    _lowerEncoder;
    private final SparkPIDController _upperPID;
    private final SparkPIDController _lowerPID;

    public ShooterFlywheelIOSparkMax()
    {
        _upperMotor = new CANSparkMax(Constants.CAN.SHOOTER_FLYWHEEL_UPPER, MotorType.kBrushless);
        _lowerMotor = new CANSparkMax(Constants.CAN.SHOOTER_FLYWHEEL_LOWER, MotorType.kBrushless);

        _upperMotor.restoreFactoryDefaults();
        _lowerMotor.restoreFactoryDefaults();

        _upperMotor.setCANTimeout(250);
        _lowerMotor.setCANTimeout(250);

        _upperMotor.setSmartCurrentLimit(40);
        _lowerMotor.setSmartCurrentLimit(40);

        _upperMotor.setIdleMode(IdleMode.kCoast);
        _lowerMotor.setIdleMode(IdleMode.kCoast);

        _upperEncoder = _upperMotor.getEncoder();
        _lowerEncoder = _lowerMotor.getEncoder();

        _upperPID = _upperMotor.getPIDController();
        _lowerPID = _lowerMotor.getPIDController();

        for (var pid : new SparkPIDController[] { _upperPID, _lowerPID })
        {
            pid.setP(6e-5);
            pid.setI(0);
            pid.setD(0);
            pid.setIZone(0);
            pid.setFF(0.000015);
            pid.setOutputRange(-1, 1);
        }

        _upperMotor.setCANTimeout(0);
        _lowerMotor.setCANTimeout(0);

        _upperMotor.burnFlash();
        _lowerMotor.burnFlash();
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        inputs.upperVelocity = _upperEncoder.getVelocity();
        inputs.upperVolts    = _upperMotor.getAppliedOutput() * _upperMotor.getBusVoltage();
        inputs.upperCurrent  = new double[] { _upperMotor.getOutputCurrent() };

        inputs.lowerVelocity = _lowerEncoder.getVelocity();
        inputs.lowerVolts    = _lowerMotor.getAppliedOutput() * _lowerMotor.getBusVoltage();
        inputs.lowerCurrent  = new double[] { _lowerMotor.getOutputCurrent() };
    }

    @Override
    public void setUpperVelocity(double velocity)
    {
        _upperPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setLowerVelocity(double velocity)
    {
        _lowerPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setUpperVolts(double volts)
    {
        _upperMotor.setVoltage(volts);
    }

    @Override
    public void setLowerVolts(double volts)
    {
        _lowerMotor.setVoltage(volts);
    }
}
