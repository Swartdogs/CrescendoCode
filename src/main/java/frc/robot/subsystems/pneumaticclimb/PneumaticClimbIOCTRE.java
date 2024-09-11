package frc.robot.subsystems.pneumaticclimb;

import com.ctre.phoenix.CTREJNIWrapper;
import com.ctre.phoenix6.jni.CtreJniWrapper;

import edu.wpi.first.hal.CTREPCMJNI;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticClimbIOCTRE implements PneumaticClimbIO
{
    private final Solenoid _leftClimb = new Solenoid(0, null, 0);
    private final CTREJNIWrapper _rightClimb = new CTREJNIWrapper();

    public PneumaticClimbIOCTRE()
    {
        
    }

    @Override
    public void updateInputs(PneumaticClimbIOInputs inputs)
    {
        
    }
}
