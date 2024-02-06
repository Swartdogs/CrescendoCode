package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterBed extends SubsystemBase
{
    private final ShooterBedIO                 _io;
    private final ShooterBedIOInputsAutoLogged _inputs        = new ShooterBedIOInputsAutoLogged();
    private PIDController                      _bedFeedback;
    private Rotation2d                         _angleSetpoint = null;
    private Rotation2d                         _minBedAngle   = Constants.ShooterBed.MIN_BED_ANGLE;
    private Rotation2d                         _maxBedAngle   = Constants.ShooterBed.MAX_BED_ANGLE;

    public ShooterBed(ShooterBedIO io)
    {
        _io = io;

        _bedFeedback = new PIDController(34.4, 0, 0); // FIXME: Set values, calibrate
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Shooter/Bed", _inputs);

        if (_angleSetpoint != null)
        {
            _io.setVoltage(_bedFeedback.calculate(_inputs.bedAngle.getRadians(), _angleSetpoint.getRadians()));
        }
    }

    public void setAngle(Rotation2d angleSetpoint)
    {
        _angleSetpoint = new Rotation2d(MathUtil.clamp(angleSetpoint.getRadians(), _minBedAngle.getRadians(), _maxBedAngle.getRadians()));
    }

    public void setAngleOffset(Rotation2d angleOffset)
    {
        _io.setAngleOffset(angleOffset);
    }

    public boolean isAtSetpoint()
    {
        return _bedFeedback.atSetpoint();
    }

    public void setMinAngle(Rotation2d minBedAngle)
    {
        _minBedAngle = minBedAngle;
    }

    public void setMaxAngle(Rotation2d maxBedAngle)
    {
        _maxBedAngle = maxBedAngle;
    }

    public Rotation2d getBedAngle()
    {
        return _inputs.bedAngle;
    }
}
