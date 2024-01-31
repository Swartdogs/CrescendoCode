// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.notepath;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Notepath extends SubsystemBase
{
    private NotepathIO               _io;
    private NotepathInputsAutoLogged _inputs               = new NotepathInputsAutoLogged();
    private double                   _intakePickupVoltage  = Constants.Notepath.NOTEPATH_INTAKE_PICKUP_VOLTAGE;
    private double                   _notepathFeedVoltage  = Constants.Notepath.NOTEPATH_VOLTAGE;
    private double                   _shooterPickupVoltage = Constants.Notepath.NOTEPATH_SHOOTER_PICKUP_VOLTAGE;

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
        _io.setVoltage(_intakePickupVoltage);
    }

    public void setFeedOn()
    {
        _io.setVoltage(_notepathFeedVoltage);
    }

    public void setNotepathShooterPickupOn()
    {
        _io.setVoltage(_shooterPickupVoltage);
    }

    public void setOff()
    {
        _io.setVoltage(0);
    }

    public void setReverse()
    {
        _io.setVoltage(-_notepathFeedVoltage);
    }

    public void setNotepathIntakePickupVoltage(double volts)
    {
        _intakePickupVoltage = volts;
    }

    public void setNotepathFeedVoltage(double volts)
    {
        _notepathFeedVoltage = volts;
    }

    public void setNotepathShooterPickupVoltage(double volts)
    {
        _shooterPickupVoltage = volts;
    }

    public double getVoltage()
    {
        return _notepathFeedVoltage;
    }
}
