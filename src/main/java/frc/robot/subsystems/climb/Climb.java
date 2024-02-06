// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.GyroIONavX2;

import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase
{
    private final ClimbIO                 _io;
    private final ClimbIOInputsAutoLogged _inputs = new ClimbIOInputsAutoLogged();
    private final PIDController           _tiltPID;
    private final PIDController           _leftPID;
    private final PIDController           _rightPID;
    
    // Average height that the two arms should be set to
    private Double _desiredHeight;
    
    private double _climbMinExtension  = Constants.Climb.MIN_EXTENSION; // TODO: tune value
    private double _climbMaxExtension  = Constants.Climb.MAX_EXTENSION; // TODO: tune value
    
    GyroIONavX2 _gyroIONavX2;
    
    public Climb(ClimbIO io)
    {
        _io = io;

        _tiltPID  = new PIDController(0, 0, 0); // TODO: tune values
        _leftPID  = new PIDController(0, 0, 0);
        _rightPID = new PIDController(0, 0, 0);
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Climb", _inputs);

        if (_desiredHeight != null)
        {
            double heightAdjustment = _tiltPID.calculate(_gyroIONavX2.getCurrentTilt(), 0);
            double leftSetpoint  = _desiredHeight + heightAdjustment;
            double rightSetpoint = _desiredHeight - heightAdjustment;

            _io.setVoltageLeft(_leftPID.calculate(getExtensionLeft(), leftSetpoint));
            _io.setVoltageRight(_rightPID.calculate(getExtensionRight(), rightSetpoint));
        }
    }

    public void stop()
    {
        _io.setVoltageLeft(0.0);
        _io.setVoltageRight(0.0);

        _desiredHeight = 0.0;
    }

    public void setVoltageLeft(double volts)  //TODO: Check understanding
    {
        if(_desiredHeight != null)
        {
            _desiredHeight = null;
            _io.setVoltageRight(0.0);
        }

        _io.setVoltageLeft(volts);
    }

    public void setVoltageRight(double volts)
    {
        if(_desiredHeight != null)
        {
            _desiredHeight = null;
            _io.setVoltageLeft(0.0);
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

    public void setExtensionMax(double max)
    {
        _climbMaxExtension = max;
    }

    public void setExtensionMin(double min)
    {
        _climbMinExtension = min;
    }

    public void setLeftAngleOffset(double angleOffset)
    {
        _io.setLeftAngleOffset(angleOffset);
    }

    public void setRightAngleOffset(double angleOffset)
    {
        _io.setRightAngleOffset(angleOffset);
    }

    // Mainly for dashboard
    public double getExtensionLeft()
    {
        return _inputs.extensionLeft;
    }

    // Mainly for dashboard
    public double getExtensionRight()
    {
        return _inputs.extensionRight;
    }

    public void setHeight(double x)
    {
        _desiredHeight = x;
    }
}
