package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFlywheel extends SubsystemBase
{
    private ShooterFlywheelIO _flywheelIO;

    private final ShooterFlywheelIOInputsAutoLogged _flywheelInputs = new ShooterFlywheelIOInputsAutoLogged();

    private SimpleMotorFeedforward _flywheelFeedforward;
    private PIDController _upperFlywheelFeedback;
    private PIDController _lowerFlywheelFeedback;

    private Double _upperVelocitySetpoint = null;
    private Double _lowerVelocitySetpoint = null;

    private double _maxFlywheelSpeed = Constants.ShooterFlywheel.MAX_FLYWHEEL_SPEED;

    public ShooterFlywheel(ShooterFlywheelIO flywheelIO)
    {
        _flywheelIO = flywheelIO;

        _flywheelFeedforward = new SimpleMotorFeedforward(0, 0);
        _upperFlywheelFeedback = new PIDController(0, 0, 0);// FIXME: Set values, calibrate
        _lowerFlywheelFeedback = new PIDController(0, 0, 0);

    }

    @Override
    public void periodic()
    {
        _flywheelIO.updateInputs(_flywheelInputs);
        Logger.processInputs("Shooter/Flywheel", _flywheelInputs);

        if (_upperVelocitySetpoint != null)
        {
            _flywheelIO.setUpperVoltage(_flywheelFeedforward.calculate(_flywheelInputs.upperFlywheelAppliedVolts)
                            + _upperFlywheelFeedback.calculate(_flywheelInputs.upperFlywheelAppliedVolts,
                                            _upperVelocitySetpoint));
        }

        if (_lowerVelocitySetpoint != null)
        {
            _flywheelIO.setLowerVoltage(_flywheelFeedforward.calculate(_flywheelInputs.lowerFlywheelAppliedVolts)
                            + _lowerFlywheelFeedback.calculate(_flywheelInputs.lowerFlywheelAppliedVolts,
                                            _lowerVelocitySetpoint));
        }
    }

    public void setUpperVelocity(double upperVelocitySetpoint)
    {
        if (upperVelocitySetpoint > _maxFlywheelSpeed)
        {
            upperVelocitySetpoint = _maxFlywheelSpeed;
        }
        
        _upperVelocitySetpoint = upperVelocitySetpoint;
    }

    public void setLowerVelocity(double lowerVelocitySetpoint)
    {
        if (lowerVelocitySetpoint > _maxFlywheelSpeed)
        {
            lowerVelocitySetpoint = _maxFlywheelSpeed;
        }

        _lowerVelocitySetpoint = lowerVelocitySetpoint;
    }

    public void stopUpper()
    {
        _upperVelocitySetpoint = null;
        _flywheelIO.setUpperVoltage(0);
    }

    public void stopLower()
    {
        _lowerVelocitySetpoint = null;
        _flywheelIO.setLowerVoltage(0);
    }

    public void setMaxFlywheelSpeed(double maxFlywheelSpeed)
    {
        _maxFlywheelSpeed = maxFlywheelSpeed;
    }
}
