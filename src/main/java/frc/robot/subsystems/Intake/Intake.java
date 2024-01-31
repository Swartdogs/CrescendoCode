package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase
{
    private IntakeIO                       _io;
    private final IntakeIOInputsAutoLogged _inputs         = new IntakeIOInputsAutoLogged();
    private double                         _intakeVoltage  = Constants.Intake.INTAKE_VOLTAGE;
    private double                         _outtakeVoltage = Constants.Intake.OUTTAKE_VOLTAGE;

    public Intake(IntakeIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Intake", _inputs);
        Logger.recordOutput("Intake/Commanded Voltage", Constants.Intake.INTAKE_VOLTAGE);
    }

    public void setIntakeOn()
    {
        _io.setVoltage(_intakeVoltage);
    }

    public void setIntakeOff()
    {
        _io.setVoltage(0.0);
    }

    public void setIntakeReverse()
    {
        _io.setVoltage(-_outtakeVoltage);
    }

    public void setIntakeVoltage(double intakeVoltage)
    {
        _intakeVoltage = intakeVoltage;
    }

    public void setOuttakeVoltage(double outtakeVoltage)
    {
        _outtakeVoltage = outtakeVoltage;
    }

    public double getIntakeVoltage()
    {
        return _inputs.appliedVolts;
    }
}
