package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase
{
    private final VisionIO                 _io;
    private final VisionIOInputsAutoLogged _inputs        = new VisionIOInputsAutoLogged();
    private final Drive                    _drive;
    private double                         _lastTimestamp = 0.0;

    public Vision(Drive drive, VisionIO io)
    {
        _drive = drive;
        _io    = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Vision", _inputs);

        if (_lastTimestamp != _inputs.captureTimestamp)
        {
            if (_inputs.pose != null)
            {
                _drive.addVisionMeasurement(_inputs.pose, _inputs.captureTimestamp);
            }
            _lastTimestamp = _inputs.captureTimestamp;
        }
    }
}
