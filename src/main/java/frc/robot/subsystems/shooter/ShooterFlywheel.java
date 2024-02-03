package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFlywheel extends SubsystemBase
{
    private final ShooterFlywheelIO                 _flywheelIO;
    private final ShooterFlywheelIOInputsAutoLogged _inputs           = new ShooterFlywheelIOInputsAutoLogged();
    private double                                  _maxFlywheelSpeed = Constants.ShooterFlywheel.MAX_FLYWHEEL_SPEED;

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
        _flywheelIO.setUpperVelocity(MathUtil.clamp(upperVelocitySetpoint, 0, _maxFlywheelSpeed));
    }

    public void setLowerVelocity(double lowerVelocitySetpoint)
    {
        _flywheelIO.setLowerVelocity(MathUtil.clamp(lowerVelocitySetpoint, 0, _maxFlywheelSpeed));
    }

    public void stop()
    {
        _flywheelIO.setUpperVoltage(0);
        _flywheelIO.setLowerVoltage(0);
    }

    public void setMaxFlywheelSpeed(double maxFlywheelSpeed)
    {
        _maxFlywheelSpeed = maxFlywheelSpeed;
    }

    public double getUpperFlywheelVelocity()
    {
        return _inputs.upperFlywheelVelocity;
    }

    public double getLowerFlywheelVelocity()
    {
        return _inputs.lowerFlywheelVelocity;
    }
}
