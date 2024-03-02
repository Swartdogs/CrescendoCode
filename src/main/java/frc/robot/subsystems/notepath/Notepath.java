package frc.robot.subsystems.notepath;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Notepath extends SubsystemBase
{
    public enum NotepathState
    {
        IntakeLoad, ShooterLoad, Feed, Off
    }

    private final NotepathIO               _io;
    private final NotepathInputsAutoLogged _inputs           = new NotepathInputsAutoLogged();
    private double                         _intakeLoadVolts  = Constants.Notepath.NOTEPATH_INTAKE_PICKUP_PERCENT_OUTPUT * Constants.General.MOTOR_VOLTAGE;
    private double                         _feedVolts        = Constants.Notepath.NOTEPATH_FEED_PERCENT_OUTPUT * Constants.General.MOTOR_VOLTAGE;
    private double                         _shooterLoadVolts = -Constants.Notepath.NOTEPATH_SHOOTER_PICKUP_PERCENT_OUTPUT * Constants.General.MOTOR_VOLTAGE;
    private boolean                        _hasNote          = true; // TODO: CHANGE if needed

    public Notepath(NotepathIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Notepath", _inputs);
    }

    public void set(NotepathState state)
    {
        _io.setVolts(switch (state)
        {
            case IntakeLoad -> _intakeLoadVolts;
            case ShooterLoad -> _shooterLoadVolts;
            case Feed -> _feedVolts;
            case Off -> 0;
            default -> 0;
        });
    }

    public void setIntakeLoadSpeed(double speed)
    {
        _intakeLoadVolts = Constants.General.MOTOR_VOLTAGE * speed;
    }

    public void setFeedSpeed(double speed)
    {
        _feedVolts = Constants.General.MOTOR_VOLTAGE * speed;
    }

    public void setShooterLoadSpeed(double speed)
    {
        _shooterLoadVolts = -Constants.General.MOTOR_VOLTAGE * speed;
    }

    public double getSpeed()
    {
        return _inputs.leaderVolts / Constants.General.MOTOR_VOLTAGE;
    }

    public boolean sensorTripped()
    {
        return _inputs.sensorTripped;
    }

    public void setHasNote(boolean hasNote)
    {
        _hasNote = hasNote;
    }

    @AutoLogOutput(key = "Notepath/HasNote")
    public boolean hasNote()
    {
        return _hasNote;
    }
}
