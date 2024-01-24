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
    private PIDController _flywheelFeedback;

    private Rotation2d _angleSetpoint = null;
    private Double _voltageSetpoint = null;

    public Shooter(ShooterBedIO bedIO, ShooterFlywheelIO flywheelIO)
    {
        _bedIO = bedIO;
        _flywheelIO = flywheelIO;

        _flywheelFeedforward = new SimpleMotorFeedforward(0, 0);
        _bedFeedback = new PIDController(0, 0, 0); // FIXME: Set values, calibrate
        _flywheelFeedback = new PIDController(0, 0, 0);
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

        if (_voltageSetpoint != null)
        {
            _flywheelIO.setVoltage(_flywheelFeedforward.calculate(_flywheelInputs.flywheelAppliedVolts)
                            + _flywheelFeedback.calculate(_flywheelInputs.flywheelAppliedVolts, _voltageSetpoint));
        }
    }

    public void setAngle(Rotation2d angleSetpoint)
    {
        _angleSetpoint = angleSetpoint;
    }

    public void setVoltage(double voltageSetpoint)
    {
        _voltageSetpoint = voltageSetpoint;
    }
}
