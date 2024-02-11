package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFlywheel extends SubsystemBase
{
    private final ShooterFlywheelIO                 _flywheelIO;
    private final ShooterFlywheelIOInputsAutoLogged _inputs                = new ShooterFlywheelIOInputsAutoLogged();
    private double                                  _maxFlywheelSpeed      = Constants.ShooterFlywheel.MAX_FLYWHEEL_SPEED * Constants.General.MAX_NEO_SPEED;
    private double                                  _flywheelIntakeVoltage = Constants.ShooterFlywheel.FLYWHEEL_INTAKE_SPEED * Constants.General.MOTOR_VOLTAGE;
    private Double                                  _upperVelocitySetpoint = null;
    private Double                                  _lowerVelocitySetpoint = null;
    private double _velocityRange = Constants.ShooterFlywheel.VELOCITY_RANGE;

    public ShooterFlywheel(ShooterFlywheelIO flywheelIO)
    {
        _flywheelIO = flywheelIO;
    }

    @Override
    public void periodic()
    {
        _flywheelIO.updateInputs(_inputs);
        Logger.processInputs("Shooter/Flywheel", _inputs);
    }

    public void setUpperVelocity(double upperVelocitySetpoint)
    {
        _upperVelocitySetpoint = MathUtil.clamp(upperVelocitySetpoint, 0, _maxFlywheelSpeed);
        _flywheelIO.setUpperVelocity(_upperVelocitySetpoint);
    }

    public void setLowerVelocity(double lowerVelocitySetpoint)
    {
        _lowerVelocitySetpoint = MathUtil.clamp(lowerVelocitySetpoint, 0, _maxFlywheelSpeed);
        _flywheelIO.setLowerVelocity(_lowerVelocitySetpoint);
    }

    public void stop()
    {
        _upperVelocitySetpoint = null;
        _lowerVelocitySetpoint = null;

        _flywheelIO.setUpperVoltage(0);
        _flywheelIO.setLowerVoltage(0);
    }

    public void flywheelIntakeOn()
    {
        _upperVelocitySetpoint = null;
        _lowerVelocitySetpoint = null;

        _flywheelIO.setUpperVoltage(-_flywheelIntakeVoltage);
        _flywheelIO.setLowerVoltage(-_flywheelIntakeVoltage);
    }

    public void setMaxFlywheelSpeed(double maxFlywheelSpeed)
    {
        _maxFlywheelSpeed = maxFlywheelSpeed * Constants.General.MAX_NEO_SPEED;
    }

    public void setFlywheelIntakeSpeed(double flywheelIntakeSpeed)
    {
        _flywheelIntakeVoltage = flywheelIntakeSpeed * Constants.General.MOTOR_VOLTAGE;
    }

    public double getUpperFlywheelVelocity()
    {
        return _inputs.upperFlywheelVelocity;
    }

    public double getLowerFlywheelVelocity()
    {
        return _inputs.lowerFlywheelVelocity;
    }

    public boolean upperAtSpeed()
    {
        if (_upperVelocitySetpoint == null)
        {
            return false;
        }
    
        return Math.abs(_upperVelocitySetpoint - getUpperFlywheelVelocity()) <= _upperVelocitySetpoint * _velocityRange;
    }
    
    public boolean lowerAtSpeed()
    {
        if (_lowerVelocitySetpoint == null)
        {
            return false;
        }
    
        return Math.abs(_lowerVelocitySetpoint - getLowerFlywheelVelocity()) <= _lowerVelocitySetpoint * _velocityRange;
    }

    public boolean atSpeed()
    {
        return upperAtSpeed() && lowerAtSpeed();
    }

    public boolean isShooting()
    {
        return _upperVelocitySetpoint != null && _lowerVelocitySetpoint != null && _upperVelocitySetpoint > 0 && _lowerVelocitySetpoint > 0;
    }

    public void setVelocityRange(double velocityRange)
{
    _velocityRange = velocityRange;
}
}
