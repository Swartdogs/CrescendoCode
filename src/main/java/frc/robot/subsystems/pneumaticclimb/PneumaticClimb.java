package frc.robot.subsystems.pneumaticclimb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticClimb extends SubsystemBase
{
    private final PneumaticClimbIO                 _io;
    private final PneumaticClimbIOInputsAutoLogged _inputs = new PneumaticClimbIOInputsAutoLogged();

    public PneumaticClimb(PneumaticClimbIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Gyro", _inputs);
    }

    public void set(boolean state)
    {
        _io.setLeft(state);
        _io.setRight(state);
    }
}
