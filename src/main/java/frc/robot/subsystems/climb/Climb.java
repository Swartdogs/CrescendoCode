// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase
{
    private final ClimbIO _io;

    private final ClimbIOInputsAutoLogged _inputs = new ClimbIOInputsAutoLogged();

    private final PIDController _climbFeedbackLeft;
    private final PIDController _climbFeedbackRight;

    private Double _climbSetpointLeft = null;
    private Double _climbSetpointRight = null;

    public Climb(ClimbIO io)
    {
	_io = io;

	_climbFeedbackLeft = new PIDController(0, 0, 0); // TODO: tune values
	_climbFeedbackRight = new PIDController(0, 0, 0);
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
	_climbSetpointLeft = setpoint;
    }

    public void setSetpointRight(double setpoint)
    {
	_climbSetpointRight = setpoint;
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
}
