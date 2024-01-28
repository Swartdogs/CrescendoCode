// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notepath;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Notepath extends SubsystemBase
{
    private NotepathIO _io;
    private NotepathInputsAutoLogged _inputs = new NotepathInputsAutoLogged();
    private double _notepathVoltage = Constants.Notepath.NOTEPATH_VOLTAGE;

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

    public void setOn()
    {
        _io.setVoltage(_notepathVoltage);
    }

    public void setOff()
    {
        _io.setVoltage(0);
    }

    public void setReverse()
    {
        _io.setVoltage(-_notepathVoltage);
    }

    public void setNotepathVoltage(double volts)
    {
        _notepathVoltage = volts;
    }

    public double getVoltage()
    {
        return _notepathVoltage;
    }

}
