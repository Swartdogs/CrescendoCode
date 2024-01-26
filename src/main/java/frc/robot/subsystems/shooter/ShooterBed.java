package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterBed extends SubsystemBase
{
    private ShooterBedIO _bedIO;

    private final ShooterBedIOInputsAutoLogged _bedInputs = new ShooterBedIOInputsAutoLogged();

    private PIDController _bedFeedback;
 
    private Rotation2d _angleSetpoint = null;
    private Rotation2d _minBedAngle = Constants.ShooterBed.MIN_BED_ANGLE;
    private Rotation2d _maxBedAngle = Constants.ShooterBed.MAX_BED_ANGLE;

    public ShooterBed(ShooterBedIO bedIO)
    {
        _bedIO = bedIO;

        _bedFeedback = new PIDController(0, 0, 0); // FIXME: Set values, calibrate
    }

    @Override
    public void periodic()
    {
        _bedIO.updateInputs(_bedInputs);
        Logger.processInputs("Shooter/Bed", _bedInputs);

        if (_angleSetpoint != null)
        {
            _bedIO.setVoltage(_bedFeedback.calculate(_bedInputs.bedAngle.getRadians(), _angleSetpoint.getRadians()));
        }
    }

    public void setAngle(Rotation2d angleSetpoint)
    {
        if (angleSetpoint.getRadians() > _maxBedAngle.getRadians())
        {
            angleSetpoint = _maxBedAngle;
        }
        else if (angleSetpoint.getRadians() < _minBedAngle.getRadians()) 
        {
            angleSetpoint = _minBedAngle;
        }

        _angleSetpoint = angleSetpoint;
    }

    public void setAngleOffset(Rotation2d angleOffset)
    {
        _bedIO.setAngleOffset(angleOffset);
    }

    public void setMinAngle(Rotation2d minBedAngle)
    {
        _minBedAngle = minBedAngle;
    }

    public void setMaxAngle(Rotation2d maxBedAngle)
    {
        _maxBedAngle = maxBedAngle;
    }
}
