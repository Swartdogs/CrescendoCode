package frc.robot.subsystems.pneumaticclimb;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;

public class PneumaticClimbIOSim implements PneumaticClimbIO
{
    private final SolenoidSim _leftClimb  = new SolenoidSim(PneumaticsModuleType.CTREPCM, 0);
    private final SolenoidSim _rightClimb = new SolenoidSim(PneumaticsModuleType.CTREPCM, 1);

    @Override
    public void updateInputs(PneumaticClimbIOInputs inputs)
    {
        inputs._leftExtended  = _leftClimb.getOutput();
        inputs._rightExtended = _rightClimb.getOutput();
    }

    @Override
    public void setLeft(boolean state)
    {
        _leftClimb.setOutput(state);
    }

    @Override
    public void setRight(boolean state)
    {
        _rightClimb.setOutput(state);
    }
}
