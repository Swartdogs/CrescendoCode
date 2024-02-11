package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMax implements IntakeIO
{
    private final CANSparkMax _motor;

    public IntakeIOSparkMax()
    {
        _motor = new CANSparkMax(Constants.CAN.INTAKE, MotorType.kBrushless);

        _motor.restoreFactoryDefaults();

        _motor.setCANTimeout(250);

        _motor.setInverted(true);
        _motor.setSmartCurrentLimit(20);

        _motor.setCANTimeout(0);

        _motor.burnFlash();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        inputs.motorVolts = _motor.getAppliedOutput() * _motor.getBusVoltage();
        inputs.motorCurrent  = new double[] { _motor.getOutputCurrent() };
    }

    @Override
    public void setVolts(double volts)
    {
        _motor.setVoltage(volts);
    }
}
