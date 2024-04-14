package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Utilities;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase
{
    private final VisionIO                 _io;
    private final VisionIOInputsAutoLogged _inputs        = new VisionIOInputsAutoLogged();
    private final Drive                    _drive;
    private final PIDController            _rotatePID;
    private double                         _maxSpeed      = 0.3;
    private double                         _lastTimestamp = 0.0;

    public Vision(Drive drive, VisionIO io)
    {
        _drive = drive;
        _io    = io;

        _rotatePID = new PIDController(0.026, 0, 0);
        _rotatePID.enableContinuousInput(-180, 180);
        _rotatePID.setTolerance(2);
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Vision", _inputs);

        if (Constants.Vision.ENABLE_POSE_CORRECTION)
        {
            if (!DriverStation.isAutonomous() && _lastTimestamp != _inputs.captureTimestamp)
            {
                if (_inputs.hasPose != false)
                {
                    _drive.addVisionMeasurement(_inputs.pose, _inputs.captureTimestamp);
                }

                _lastTimestamp = _inputs.captureTimestamp;
            }
        }
    }

    @AutoLogOutput(key = "Vision/Sees Speaker")
    public boolean seesSpeaker()
    {
        int     centerSpeakerTargetId      = Utilities.isBlueAlliance() ? 7 : 4;
        boolean seenTargetsIncludesSpeaker = false;

        for (int id : _inputs.targetIds)
        {
            if (id == centerSpeakerTargetId)
            {
                seenTargetsIncludesSpeaker = true;
                break;
            }
        }

        return seenTargetsIncludesSpeaker;
    }

    @AutoLogOutput(key = "Vision/Yaw to Speaker")
    public double getYawToSpeaker()
    {
        // First find the index of the target in our inputs array. This is
        // to ensure we're getting the right yaw from the right target
        int centerSpeakerTargetId = Utilities.isBlueAlliance() ? 7 : 4;
        int targetIndex           = _inputs.targetIds.length;

        for (int i = 0; i < _inputs.targetIds.length; i++)
        {
            if (_inputs.targetIds[i] == centerSpeakerTargetId)
            {
                targetIndex = i;
                break;
            }
        }

        // Next, find the corresponding yaw. Default to 0 if we can't find it
        double yaw = 0;

        if (targetIndex < _inputs.targetIds.length)
        {
            yaw = _inputs.targetYaws[targetIndex];
        }

        return yaw;
    }

    public void setRotateSpeed(double maxSpeed)
    {
        _maxSpeed = Math.abs(maxSpeed);
    }

    public double rotateExecute()
    {
        double rawSpeed = 0;

        if (seesSpeaker())
        {
            rawSpeed = _rotatePID.calculate(getYawToSpeaker(), Constants.Vision.ALIGNMENT_OFFSET);
        }

        return MathUtil.clamp(rawSpeed, -_maxSpeed, _maxSpeed);
    }

    public boolean rotateIsFinished()
    {
        return _rotatePID.atSetpoint();
    }
}
