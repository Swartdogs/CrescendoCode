package frc.robot.subsystems.pneumaticclimb;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticClimbIOCTRE implements PneumaticClimbIO
{
    private final Solenoid _leftClimb  = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid _rightClimb = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    @Override
    public void updateInputs(PneumaticClimbIOInputs inputs)
    {
        inputs._leftExtended  = _leftClimb.get();
        inputs._rightExtended = _rightClimb.get();
    }

    @Override
    public void setLeft(boolean state)
    {
        _leftClimb.set(state);
    }

    @Override
    public void setRight(boolean state)
    {
        _rightClimb.set(state);
    }
}
