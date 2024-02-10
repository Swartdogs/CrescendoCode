package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMax implements IntakeIO
{
    private final CANSparkMax _intakeMotor;

    public IntakeIOSparkMax()
    {
        _intakeMotor = new CANSparkMax(Constants.CAN.INTAKE, MotorType.kBrushless);

        _intakeMotor.restoreFactoryDefaults();

        _intakeMotor.setCANTimeout(250);

        _intakeMotor.setInverted(true);
        _intakeMotor.setSmartCurrentLimit(20);

        _intakeMotor.setCANTimeout(0);

        _intakeMotor.burnFlash();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        inputs.motorVolts = _intakeMotor.getAppliedOutput() * _intakeMotor.getBusVoltage();
        inputs.motorCurrent  = new double[] { _intakeMotor.getOutputCurrent() };
    }

    @Override
    public void setVoltage(double volts)
    {
        _intakeMotor.setVoltage(volts);
    }
}
