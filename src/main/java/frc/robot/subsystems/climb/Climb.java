// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.Gyro;

import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase
{
    private final Gyro                    _gyro;
    private final ClimbIO                 _io;
    private final ClimbIOInputsAutoLogged _inputs            = new ClimbIOInputsAutoLogged();
    private final PIDController           _tiltPID;
    private final PIDController           _leftPID;
    private final PIDController           _rightPID;
    private Double                        _desiredHeight;
    private double                        _climbMinExtension = Constants.Climb.MIN_EXTENSION; // TODO: tune value
    private double                        _climbMaxExtension = Constants.Climb.MAX_EXTENSION; // TODO: tune value

    public Climb(Gyro gyro, ClimbIO io)
    {
        _gyro = gyro;

        _io = io;

        _tiltPID  = new PIDController(12, 0, 0); // TODO: tune values
        _leftPID  = new PIDController(12, 0, 0);
        _rightPID = new PIDController(12, 0, 0);
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Climb", _inputs);

        if (_desiredHeight != null)
        {
            double heightAdjustment = _tiltPID.calculate(_gyro.getRollPosition().getDegrees(), 0);
            double leftSetpoint     = MathUtil.clamp(_desiredHeight + heightAdjustment, _climbMinExtension, _climbMaxExtension);
            double rightSetpoint    = MathUtil.clamp(_desiredHeight - heightAdjustment, _climbMinExtension, _climbMaxExtension);

            _io.setVoltageLeft(_leftPID.calculate(getExtensionLeft(), leftSetpoint));
            _io.setVoltageRight(_rightPID.calculate(getExtensionRight(), rightSetpoint));
        }
    }

    public void stop()
    {
        _io.setVoltageLeft(0.0);
        _io.setVoltageRight(0.0);

        _desiredHeight = null;
    }

    public void setExtensionMax(double max)
    {
        _climbMaxExtension = max;
    }

    public void setExtensionMin(double min)
    {
        _climbMinExtension = min;
    }

    public void setVoltageLeft(double volts)
    {
        if (_desiredHeight != null)
        {
            _desiredHeight = null;
            _io.setVoltageRight(0.0);
        }

        if ((_inputs.extensionLeft >= _climbMaxExtension && volts > 0) || (_inputs.extensionLeft <= _climbMinExtension && volts < 0))
        {
            volts = 0;
        }

        _io.setVoltageLeft(volts);
    }

    public void setVoltageRight(double volts)
    {
        if (_desiredHeight != null)
        {
            _desiredHeight = null;
            _io.setVoltageLeft(0.0);
        }

        if ((_inputs.extensionRight >= _climbMaxExtension && volts > 0) || (_inputs.extensionRight <= _climbMinExtension && volts < 0))
        {
            volts = 0;
        }

        _io.setVoltageRight(volts);
    }

    public boolean isAtLeftSetpoint()
    {
        return _leftPID.atSetpoint();
    }

    public boolean isAtRightSetpoint()
    {
        return _rightPID.atSetpoint();
    }

    public void setLeftAngleOffset(double angleOffset)
    {
        _io.setLeftOffset(angleOffset);
    }

    public void setRightAngleOffset(double angleOffset)
    {
        _io.setRightOffset(angleOffset);
    }

    public double getExtensionLeft()
    {
        return _inputs.extensionLeft;
    }
    
    public double getExtensionRight()
    {
        return _inputs.extensionRight;
    }

    public void setHeight(double height)
    {
        _desiredHeight = height;
    }
}
