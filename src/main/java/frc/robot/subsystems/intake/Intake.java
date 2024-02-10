package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase
{
    private final IntakeIO                 _io;
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

    public void set(Value value)
    {
        double voltage = Constants.General.MOTOR_VOLTAGE;

        switch (value)
        {
            case kOn:
            case kForward:
                voltage *= _intakePercentOutput;
                break;

            case kReverse:
                voltage *= -_outtakePercentOutput;
                break;

            case kOff:
            default:
                voltage = 0;
                break;
        }

        _io.setVoltage(voltage);
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
        return _inputs.motorVolts / Constants.General.MOTOR_VOLTAGE;
    }
}
