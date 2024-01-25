package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMax implements IntakeIO
{
    private CANSparkMax _intakeSparkMax;

    public IntakeIOSparkMax(int canId)
    {
        _intakeSparkMax = new CANSparkMax(canId, MotorType.kBrushless);

        _intakeSparkMax.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        inputs.appliedVolts = _intakeSparkMax.getAppliedOutput() * _intakeSparkMax.getBusVoltage();
        inputs.currentAmps = new double[]
        { _intakeSparkMax.getOutputCurrent() };
    }

    @Override
    public void setVoltage(double volts)
    {
        _intakeSparkMax.setVoltage(volts);
    }
}
