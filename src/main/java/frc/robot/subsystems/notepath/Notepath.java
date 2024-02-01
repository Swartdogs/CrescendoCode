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
    private double                   _intakePickupSpeedPercent  = Constants.Notepath.NOTEPATH_INTAKE_PICKUP_SPEED_PERCENT;
    private double                   _notepathFeedSpeedPercent  = Constants.Notepath.NOTEPATH_FEED_SPEED_PERCENT;
    private double                   _shooterPickupSpeedPercent = Constants.Notepath.NOTEPATH_SHOOTER_PICKUP_SPEED_PERCENT;

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
        _io.setVoltage(_intakePickupSpeedPercent * Constants.MOTOR_VOLTAGE);
    }

    public void setFeedOn()
    {
        _io.setVoltage(_notepathFeedSpeedPercent * Constants.MOTOR_VOLTAGE);
    }

    public void setNotepathShooterPickupOn()
    {
        _io.setVoltage(_shooterPickupSpeedPercent * Constants.MOTOR_VOLTAGE);
    }

    public void setOff()
    {
        _io.setVoltage(0);
    }

    public void setReverse()
    {
        _io.setVoltage(-_notepathFeedSpeedPercent * Constants.MOTOR_VOLTAGE);
    }

    public void setNotepathIntakePickupSpeedPercent(double percent)
    {
        _intakePickupSpeedPercent = percent;
    }

    public void setNotepathFeedSpeedPercent(double percent)
    {
        _notepathFeedSpeedPercent = percent;
    }

    public void setNotepathShooterPickupSpeedPercent(double percent)
    {
        _shooterPickupSpeedPercent = percent;
    }

    public double getSpeedPercent()
    {
        return _notepathFeedSpeedPercent;
    }
}
