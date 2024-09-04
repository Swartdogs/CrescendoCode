package frc.robot.subsystems.pneumaticclimb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PnuematicClimb extends SubsystemBase
{
    private final PneumaticClimbIO                 _io;
    private final PnuematicClimbIOInputsAutoLogged _inputs = new PneumaticClimbIOInputsAutoLogged();

    public PnuematicClimb(PneumaticClimbIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Gyro", _inputs);
    }
}
