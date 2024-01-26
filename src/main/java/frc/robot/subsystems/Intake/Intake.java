package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase
{
    private IntakeIO _io;
    private final IntakeIOInputsAutoLogged _inputs = new IntakeIOInputsAutoLogged();
    private double _voltage = Constants.Intake.INTAKE_VOLTAGE;

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
        _io.setVoltage(_voltage);
    }

    public void setIntakeOff()
    {
        _io.setVoltage(0.0);
    }

    public void setIntakeReverse()
    {
        _io.setVoltage(-_voltage);
    }
}
