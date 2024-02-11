package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase
{
    public enum IntakeState
    {
        On, Off, Reverse
    }

    private final IntakeIO                 _io;
    private final IntakeIOInputsAutoLogged _inputs   = new IntakeIOInputsAutoLogged();
    private double                         _inVolts  = Constants.General.MOTOR_VOLTAGE * Constants.Intake.INTAKE_DEFAULT_PERCENT_OUTPUT;
    private double                         _outVolts = Constants.General.MOTOR_VOLTAGE * Constants.Intake.OUTTAKE_DEFAULT_PERCENT_OUTPUT;

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

    public void set(IntakeState state)
    {
        _io.setVolts(switch (state)
        {
            case On -> _inVolts;
            case Reverse -> _outVolts;
            case Off -> 0;
            default -> 0;
        });
    }

    public void setInSpeed(double speed)
    {
        _inVolts = Constants.General.MOTOR_VOLTAGE * speed;
    }

    public void setOutSpeed(double speed)
    {
        _outVolts = -Constants.General.MOTOR_VOLTAGE * speed;
    }

    public double getSpeed()
    {
        return _inputs.motorVolts / Constants.General.MOTOR_VOLTAGE;
    }
}
