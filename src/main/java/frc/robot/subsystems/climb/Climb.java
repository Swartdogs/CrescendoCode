// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

public class Climb extends SubsystemBase
{
    private final ClimbIO                 _io;
    private final ClimbIOInputsAutoLogged _inputs = new ClimbIOInputsAutoLogged();
    
    private final PIDController           _climbFeedbackLeft;
    private final PIDController           _climbFeedbackRight;

    private final PIDController _tiltPID;
    private final PIDController _leftPID;
    private final PIDController _rightPID;

    private final AHRS _gyro;

    // Average height that the two arms should be set to
    private double _desiredHeight;

    // Finds the value that the robot needs to adjust by in order to be level, based on the gyro angle 
    private double _heightAdjustment = 0.0;

    private double _leftHeight;
    private double _rightHeight;

    private Double _climbSetpointLeft = null;
    private Double _climbSetpointRight = null;

    private double _climbMaxExtension = Constants.Climb.MAX_EXTENSION; // TODO: tune value
    private double _climbMinExtension = Constants.Climb.MIN_EXTENSION; // TODO: tune value

    public Climb(ClimbIO io)
    {
        _io = io;

        _climbFeedbackLeft  = new PIDController(0, 0, 0); // TODO: tune values
        _climbFeedbackRight = new PIDController(0, 0, 0);

        // Intilizes the PID controllers, need to set the actual values
        _tiltPID  = new PIDController(0, 0, 0); //TODO: Set values
        _leftPID  = new PIDController(0, 0, 0);
        _rightPID = new PIDController(0, 0, 0);

        _gyro = new AHRS(Port.kMXP);

        // Takes the desired position of both arms, and takes the needed adjustment calculated from the tilt to set the position - setpoint for both arms
        _leftHeight  = _desiredHeight + _heightAdjustment;
        _rightHeight = _desiredHeight - _heightAdjustment;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Climb", _inputs);

        if (_climbSetpointLeft != null)
        {
            _io.setVoltageLeft(_climbFeedbackLeft.calculate(_inputs.extensionLeft, _climbSetpointLeft));
        }

        if (_climbSetpointRight != null)
        {
            _io.setVoltageRight(_climbFeedbackRight.calculate(_inputs.extensionRight, _climbSetpointRight));
        }

        // Takes the pid controller and gives it the current value and the setpoint
        _heightAdjustment = _tiltPID.calculate(getCurrentTilt(), 0);

        // Gets the current position of the left and the right arms, and sets it to the desired position based on the tilt input, also applies this voltage
        _io.setAlgorithmVoltageLeft(_leftPID.calculate(getExtensionLeft(), _leftHeight)); 
        _io.setAlgorithmVoltageRight(_rightPID.calculate(getExtensionRight(), _rightHeight)); // TODO: Check if it actually runs it
    }

    public void stop()
    {
        _io.setVoltageLeft(0.0);
        _io.setVoltageRight(0.0);

        _climbSetpointLeft  = null;
        _climbSetpointRight = null;

        setLockState(true);
    }

    public void setLockState(boolean enabled)
    {
        _io.setLockStateLeft(enabled, _inputs);
        _io.setLockStateRight(enabled, _inputs);
    }

    public void setSetpointLeft(double setpoint)
    {
        _climbSetpointLeft = MathUtil.clamp(setpoint, _climbMinExtension, _climbMaxExtension);
    }

    public void setSetpointRight(double setpoint)
    {
        _climbSetpointRight = MathUtil.clamp(setpoint, _climbMinExtension, _climbMaxExtension);
    }

    public void setVoltageLeft(double volts)
    {
        _climbSetpointLeft = null;

        _io.setLockStateLeft(Math.abs(volts) <= 0.12, _inputs); // TODO: Change number to constants

        _io.setVoltageLeft(volts);
    }

    public void setVoltageRight(double volts)
    {
        _climbSetpointRight = null;

        _io.setLockStateRight(Math.abs(volts) <= 0.12, _inputs); // TODO: Change number to constants

        _io.setVoltageRight(volts);
    }

    public boolean isAtLeftSetpoint()
    {
        return _climbFeedbackLeft.atSetpoint();
    }

    public boolean isAtRightSetpoint()
    {
        return _climbFeedbackRight.atSetpoint();
    }

    public void setExtensionMax(double max)
    {
        _climbMaxExtension = max;
    }

    public void setExtensionMin(double min)
    {
        _climbMinExtension = min;
    }

    public double getExtensionLeft()
    {
        return _inputs.extensionLeft;
    }

    public double getExtensionRight()
    {
        return _inputs.extensionRight;
    }

    public void setHeight(double x)
    {
        _desiredHeight = x;
    }

    // Returns the current angle of the gyroscope
    public double getCurrentTilt()
    {
        return _gyro.getAngle(); // TODO: Check if this needs to be somewhere in IO 
    }
}
