// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.notepath;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Notepath extends SubsystemBase
{
    private NotepathIO               _io;
    private NotepathInputsAutoLogged _inputs                     = new NotepathInputsAutoLogged();
    private double                   _intakePickupPercentOutput  = Constants.Notepath.NOTEPATH_INTAKE_PICKUP_PERCENT_OUTPUT;
    private double                   _notepathFeedPercentOutput  = Constants.Notepath.NOTEPATH_FEED_PERCENT_OUTPUT;
    private double                   _shooterPickupPercentOutput = Constants.Notepath.NOTEPATH_SHOOTER_PICKUP_PERCENT_OUTPUT;
    private boolean                  _hasNote                    = false;

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

    public void setNotepathIntakePickupOn()
    {
        _io.setVoltage(_intakePickupPercentOutput * Constants.General.MOTOR_VOLTAGE);
    }

    public void setFeedOn()
    {
        _io.setVoltage(_notepathFeedPercentOutput * Constants.General.MOTOR_VOLTAGE);
    }

    public void setNotepathShooterPickupOn()
    {
        _io.setVoltage(_shooterPickupPercentOutput * Constants.General.MOTOR_VOLTAGE);
    }

    public void setOff()
    {
        _io.setVoltage(0);
    }

    public void setReverse()
    {
        _io.setVoltage(-_notepathFeedPercentOutput * Constants.General.MOTOR_VOLTAGE);
    }

    public void setNotepathIntakePickupPercentOutput(double percentOutput)
    {
        _intakePickupPercentOutput = percentOutput;
    }

    public void setNotepathFeedPercentOutput(double percentOutput)
    {
        _notepathFeedPercentOutput = percentOutput;
    }

    public void setNotepathShooterPickupPercentOutput(double percentOutput)
    {
        _shooterPickupPercentOutput = percentOutput;
    }

    public double getPercentOutput()
    {
        return _inputs.leaderNotepathAppliedVolts / Constants.General.MOTOR_VOLTAGE;
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
