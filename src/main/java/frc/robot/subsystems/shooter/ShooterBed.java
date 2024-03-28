package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterBed extends SubsystemBase
{
    public enum BedAngle
    {
        // @formatter:off
        IntakeLoad(Constants.ShooterBed.BED_INTAKE_PICKUP_ANGLE), 
        ShooterLoad(Constants.ShooterBed.BED_SHOOTER_PICKUP_ANGLE), 
        SubwooferShot(Constants.ShooterBed.BED_SUBWOOFER_SHOT_ANGLE),
        PodiumShot(Constants.ShooterBed.BED_PODIUM_SHOT_ANGLE),
        AmpShot(Constants.ShooterBed.BED_AMP_SHOT_ANGLE),
        TrapShot(Constants.ShooterBed.Bed_TRAP_SHOT_ANGLE),
        ClimbVertical(Constants.ShooterBed.BED_CLIMB_VERTICAL_ANGLE),
        Stow(Constants.ShooterBed.BED_STOW_ANGLE);
        // @formatter:on

        private Rotation2d _angle;

        private BedAngle(Rotation2d angle)
        {
            _angle = angle;
        }

        public Rotation2d getAngle()
        {
            return _angle;
        }

        public void setAngle(Rotation2d angle)
        {
            _angle = angle;
        }
    }

    private final ShooterBedIO                 _io;
    private final ShooterBedIOInputsAutoLogged _inputs        = new ShooterBedIOInputsAutoLogged();
    private final PIDController                _bedPID;
    private Rotation2d                         _angleSetpoint = null;
    private Rotation2d                         _minAngle      = Constants.ShooterBed.MIN_BED_ANGLE;
    private Rotation2d                         _maxAngle      = Constants.ShooterBed.MAX_BED_ANGLE;

    public ShooterBed(ShooterBedIO io)
    {
        _io = io;

        _bedPID = new PIDController(220 / Math.PI, 0, 3); // FIXME: Set values, calibrate
        _bedPID.setTolerance(Units.degreesToRadians(1.5));
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Shooter/Bed", _inputs);

        if (_angleSetpoint != null)
        {
            // var feedForward = Constants.ShooterBed.BED_DOWN_MIN_VOLTS;

            // if (_angleSetpoint.getRadians() > _inputs.bedAngle.getRadians())
            // {
            // feedForward = Constants.ShooterBed.BED_UP_MIN_VOLTS;
            // }

            // if (atSetpoint())
            // {
            // feedForward = 0;
            // }

            var feedForward = Math.sin(_inputs.bedAngle.getRadians()) * Constants.ShooterBed.BED_DOWN_MIN_VOLTS;

            _io.setVolts(MathUtil.clamp(feedForward + _bedPID.calculate(_inputs.bedAngle.getRadians(), _angleSetpoint.getRadians()), -Constants.ShooterBed.MAX_BED_VOLTS, Constants.ShooterBed.MAX_BED_VOLTS));
        }

        Logger.recordOutput("Has Bed Setpoint", _angleSetpoint != null);
    }

    public void setVolts(double volts)
    {
        _angleSetpoint = null;
        _io.setVolts(volts);
    }

    public void setAngle(Rotation2d angle)
    {
        _angleSetpoint = new Rotation2d(MathUtil.clamp(angle.getRadians(), _minAngle.getRadians(), _maxAngle.getRadians()));
    }

    public void setAngle(BedAngle angle)
    {
        setAngle(angle.getAngle());
    }

    public void setAngleOffset(Rotation2d angleOffset)
    {
        _io.setOffset(angleOffset);
    }

    public boolean atSetpoint()
    {
        return _bedPID.atSetpoint();
    }

    public void setMinAngle(Rotation2d angle)
    {
        _minAngle = angle;
    }

    public void setMaxAngle(Rotation2d angle)
    {
        _maxAngle = angle;
    }

    public void setIntakeLoadAngle(Rotation2d angle)
    {
        BedAngle.IntakeLoad.setAngle(angle);
    }

    public void setShooterLoadAngle(Rotation2d angle)
    {
        BedAngle.ShooterLoad.setAngle(angle);
    }

    public Rotation2d getBedAngle()
    {
        return _inputs.bedAngle;
    }
}
