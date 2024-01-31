// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Module
{
    private final ModuleIO                 _io;
    private final ModuleIOInputsAutoLogged _inputs             = new ModuleIOInputsAutoLogged();
    private final int                      _index;
    private final SimpleMotorFeedforward   _driveFeedforward;
    private final PIDController            _driveFeedback;
    private final PIDController            _turnFeedback;
    private Rotation2d                     _angleSetpoint      = null;
    private Double                         _speedSetpoint      = null;
    private Rotation2d                     _turnRelativeOffset = null;
    private double                         _lastPositionMeters = 0.0;

    public Module(ModuleIO io, int index)
    {
        _io    = io;
        _index = index;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.AdvantageKit.CURRENT_MODE)
        {
            case REAL:
            case REPLAY:
                _driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);

                _driveFeedback = new PIDController(0.05, 0.0, 0.0);
                _turnFeedback = new PIDController(7.0, 0.0, 0.0);
                break;

            case SIM:
                _driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);

                _driveFeedback = new PIDController(0.1, 0.0, 0.0);
                _turnFeedback = new PIDController(10.0, 0.0, 0.0);
                break;

            default:
                _driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

                _driveFeedback = new PIDController(0.0, 0.0, 0.0);
                _turnFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        _turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
        setBrakeMode(true);
    }

    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(_index), _inputs);

        // On first cycle, reset relative turn encoder
        // Wait until absolute angle is nonzero in case it wasn't initialized yet
        if (_turnRelativeOffset == null && _inputs.turnAbsolutePosition.getRadians() != 0.0)
        {
            _turnRelativeOffset = _inputs.turnAbsolutePosition.minus(_inputs.turnPosition);
        }

        // Run closed loop turn control
        if (_angleSetpoint != null)
        {
            _io.setTurnVoltage(_turnFeedback.calculate(getAngle().getRadians(), _angleSetpoint.getRadians()));

            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (_speedSetpoint != null)
            {
                // Scale velocity based on turn error
                //
                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                double adjustSpeedSetpoint = _speedSetpoint * Math.cos(_turnFeedback.getPositionError());

                // Run drive controller
                double velocityRadPerSec = adjustSpeedSetpoint / Constants.Drive.WHEEL_RADIUS;
                _io.setDriveVoltage(_driveFeedforward.calculate(velocityRadPerSec) + _driveFeedback.calculate(_inputs.driveVelocityRadPerSec, velocityRadPerSec));
            }
        }
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized
     * state.
     */
    public SwerveModuleState runSetpoint(SwerveModuleState state)
    {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        var optimizedState = SwerveModuleState.optimize(state, getAngle());

        // Update setpoints, controllers run in "periodic"
        _angleSetpoint = optimizedState.angle;
        _speedSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     */
    public void runCharacterization(double volts)
    {
        // Closed loop turn control
        _angleSetpoint = new Rotation2d();

        // Open loop drive control
        _io.setDriveVoltage(volts);
        _speedSetpoint = null;
    }

    /** Disables all outputs to motors. */
    public void stop()
    {
        _io.setTurnVoltage(0.0);
        _io.setDriveVoltage(0.0);

        // Disable closed loop control for turn and drive
        _angleSetpoint = null;
        _speedSetpoint = null;
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled)
    {
        _io.setDriveBrakeMode(enabled);
        _io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle()
    {
        if (_turnRelativeOffset == null)
        {
            return new Rotation2d();
        }
        else
        {
            return _inputs.turnPosition.plus(_turnRelativeOffset);
        }
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters()
    {
        return _inputs.drivePositionRad * Constants.Drive.WHEEL_RADIUS;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec()
    {
        return _inputs.driveVelocityRadPerSec * Constants.Drive.WHEEL_RADIUS;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module position delta since the last call to this method. */
    public SwerveModulePosition getPositionDelta()
    {
        var delta = new SwerveModulePosition(getPositionMeters() - _lastPositionMeters, getAngle());
        _lastPositionMeters = getPositionMeters();
        return delta;
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the drive velocity in radians/sec. */
    public double getCharacterizationVelocity()
    {
        return _inputs.driveVelocityRadPerSec;
    }
}
