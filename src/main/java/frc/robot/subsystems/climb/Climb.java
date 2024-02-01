// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.GyroIONavX2;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

public class Climb extends SubsystemBase
{
    private final ClimbIO _io;

    private final ClimbIOInputsAutoLogged _inputs = new ClimbIOInputsAutoLogged();

    private final PIDController _climbFeedbackLeft;
    private final PIDController _climbFeedbackRight;

    private final AHRS _gyro;

    // Three PID controllers for the algorithm
    private final PIDController _tiltPID;
    private final PIDController _leftPID;
    private final PIDController _rightPID;

    // Average height that the two arms should be set to
    private double _desiredHeight = 5; // TODO: Create two functions for setting the heights, also change value
    // Finds the value that the robot needs to adjust by in order to be level, based on the gyro angle 
    private double _heightAdjustment = 0.0;

    private Double _climbSetpointLeft = null;
    private Double _climbSetpointRight = null;

    private double _climbMaxExtension = Constants.Climb.MAX_EXTENSION; // TODO: tune value
    private double _climbMinExtension = Constants.Climb.MIN_EXTENSION; // TODO: tune value

    public Climb(ClimbIO io)
    {
        _io = io;

        _climbFeedbackLeft = new PIDController(0, 0, 0); // TODO: tune values
        _climbFeedbackRight = new PIDController(0, 0, 0);

        _gyro = new AHRS(Port.kMXP);

        // Intilizes the PID controllers, need to set the actual values
        _tiltPID = new PIDController(0, 0, 0);
        _leftPID = new PIDController(0, 0, 0);
        _rightPID = new PIDController(0, 0, 0);
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
    }

    public void stop()
    {
        _io.setVoltageLeft(0.0);
        _io.setVoltageRight(0.0);

        _climbSetpointLeft = null;
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

    // Returns the current angle of the gyroscope
    public double getCurrentTilt()
    {
        return _gyro.getAngle();  
    }
}
