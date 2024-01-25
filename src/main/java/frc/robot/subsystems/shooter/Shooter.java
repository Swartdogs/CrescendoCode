package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    private ShooterBedIO _bedIO;
    private ShooterFlywheelIO _flywheelIO;

    private final ShooterBedIOInputsAutoLogged _bedInputs = new ShooterBedIOInputsAutoLogged();
    private final ShooterFlywheelIOInputsAutoLogged _flywheelInputs = new ShooterFlywheelIOInputsAutoLogged();

    private SimpleMotorFeedforward _flywheelFeedforward;
    private PIDController _bedFeedback;
    private PIDController _upperFlywheelFeedback;
    private PIDController _lowerFlywheelFeedback;

    private Rotation2d _angleSetpoint = null;
    private Double _upperVelocitySetpoint = null;
    private Double _lowerVelocitySetpoint = null;

    public Shooter(ShooterBedIO bedIO, ShooterFlywheelIO flywheelIO)
    {
        _bedIO = bedIO;
        _flywheelIO = flywheelIO;

        _flywheelFeedforward = new SimpleMotorFeedforward(0, 0);
        _bedFeedback = new PIDController(0, 0, 0); // FIXME: Set values, calibrate
        _upperFlywheelFeedback = new PIDController(0, 0, 0);
        _lowerFlywheelFeedback = new PIDController(0, 0, 0);

    }

    @Override
    public void periodic()
    {
        _bedIO.updateInputs(_bedInputs);
        _flywheelIO.updateInputs(_flywheelInputs);
        Logger.processInputs("Shooter/Bed", _bedInputs);
        Logger.processInputs("Shooter/Flywheel", _flywheelInputs);

        if (_angleSetpoint != null)
        {
            _bedIO.setVoltage(_bedFeedback.calculate(_bedInputs.bedAngle.getRadians(), _angleSetpoint.getRadians()));
        }

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

    public void setAngle(Rotation2d angleSetpoint)
    {
        _angleSetpoint = angleSetpoint;
    }

    public void setUpperVelocity(double upperVelocitySetpoint)
    {
        _upperVelocitySetpoint = upperVelocitySetpoint;
    }

    public void setLowerVelocity(double lowerVelocitySetpoint)
    {
        _lowerVelocitySetpoint = lowerVelocitySetpoint;
    }

    public void setUpperVoltage(double upperVoltage)
    {
        _upperVelocitySetpoint = null;
        _flywheelIO.setUpperVoltage(0);

    }

    public void setLowerVoltage(double lowerVoltage)
    {
        _lowerVelocitySetpoint = null;
        _flywheelIO.setLowerVoltage(0);
    }
}
