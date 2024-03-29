package frc.robot.subsystems.trap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trap extends SubsystemBase
{
    private final TrapIO                 _io;
    private final TrapIOInputsAutoLogged _inputs = new TrapIOInputsAutoLogged();

    public Trap(Trap trap, TrapIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Trap", _inputs);
    }

    public void setVolts(double volts)
    {
        _io.setVolts(volts);
    }
}
