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
    private final ClimbIOInputsAutoLogged _inputs       = new ClimbIOInputsAutoLogged();
    private final PIDController           _tiltPID;
    private final PIDController           _leftPID;
    private final PIDController           _rightPID;
    private Double                        _desiredHeight;
    private double                        _minExtension = Constants.Climb.MIN_EXTENSION; // TODO: tune value
    private double                        _maxExtension = Constants.Climb.MAX_EXTENSION; // TODO: tune value

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
            double leftSetpoint     = MathUtil.clamp(_desiredHeight + heightAdjustment, _minExtension, _maxExtension);
            double rightSetpoint    = MathUtil.clamp(_desiredHeight - heightAdjustment, _minExtension, _maxExtension);

            _io.setLeftVolts(_leftPID.calculate(getLeftExtension(), leftSetpoint));
            _io.setRightVolts(_rightPID.calculate(getRightExtension(), rightSetpoint));
        }
    }

    public void stop()
    {
        _io.setLeftVolts(0.0);
        _io.setRightVolts(0.0);

        _desiredHeight = null;
    }

    public void setMinExtension(double min)
    {
        _minExtension = min;
    }

    public void setMaxExtension(double max)
    {
        _maxExtension = max;
    }

    public void setLeftVolts(double volts)
    {
        if (_desiredHeight != null)
        {
            _desiredHeight = null;
            _io.setRightVolts(0.0);
        }

        if ((_inputs.leftExtension >= _maxExtension && volts > 0) || (_inputs.leftExtension <= _minExtension && volts < 0))
        {
            volts = 0;
        }

        _io.setLeftVolts(volts);
    }

    public void setRightVolts(double volts)
    {
        if (_desiredHeight != null)
        {
            _desiredHeight = null;
            _io.setLeftVolts(0.0);
        }

        if ((_inputs.rightExtension >= _maxExtension && volts > 0) || (_inputs.rightExtension <= _minExtension && volts < 0))
        {
            volts = 0;
        }

        _io.setRightVolts(volts);
    }

    public boolean isAtLeftSetpoint()
    {
        return _leftPID.atSetpoint();
    }

    public boolean isAtRightSetpoint()
    {
        return _rightPID.atSetpoint();
    }

    public void setLeftOffset(double offset)
    {
        _io.setLeftOffset(offset);
    }

    public void setRightOffset(double offset)
    {
        _io.setRightOffset(offset);
    }

    public double getLeftExtension()
    {
        return _inputs.leftExtension;
    }

    public double getRightExtension()
    {
        return _inputs.rightExtension;
    }

    public void setHeight(double height)
    {
        _desiredHeight = height;
    }
}
