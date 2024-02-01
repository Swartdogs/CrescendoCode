package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase
{
    private IntakeIO                       _io;
    private final IntakeIOInputsAutoLogged _inputs               = new IntakeIOInputsAutoLogged();
    private double                         _intakePercentOutput  = Constants.Intake.INTAKE_DEFAULT_PERCENT_OUTPUT;
    private double                         _outtakePercentOutput = Constants.Intake.OUTTAKE_DEFAULT_PERCENT_OUTPUT;

    public Intake(IntakeIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Intake", _inputs);
    }

    public void setIntakeOn()
    {
        _io.setVoltage(_intakePercentOutput * Constants.General.MOTOR_VOLTAGE);
    }

    public void setIntakeOff()
    {
        _io.setVoltage(0.0);
    }

    public void setIntakeReverse()
    {
        _io.setVoltage(-_outtakePercentOutput * Constants.General.MOTOR_VOLTAGE);
    }

    public void setIntakePercentOutput(double percentOutput)
    {
        _intakePercentOutput = percentOutput;
    }

    public void setOuttakePercentOutput(double percentOutput)
    {
        _outtakePercentOutput = percentOutput;
    }

    public double getPercentOutput()
    {
        return _inputs.appliedVolts / Constants.General.MOTOR_VOLTAGE;
    }
}
