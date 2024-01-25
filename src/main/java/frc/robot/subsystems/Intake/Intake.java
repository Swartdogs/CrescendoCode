package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
    private IntakeIO _io;
    private double _intakeVoltage = 0.0;
    private final IntakeIOInputsAutoLogged _inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Intake", _inputs);
        Logger.recordOutput("Intake/Commanded Voltage", _intakeVoltage);
        _io.setVoltage(_intakeVoltage);
    }

    public void setIntakeVoltage(double intakeVoltage)
    {
        _intakeVoltage = intakeVoltage;
    }
}
