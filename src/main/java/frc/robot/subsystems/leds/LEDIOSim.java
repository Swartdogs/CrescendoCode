package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import frc.robot.Constants;

public class LEDIOSim implements LEDIO
{
    private AddressableLEDSim    _ledSim;
    private AddressableLEDBuffer _ledBuffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);

    public LEDIOSim()
    {

    }
}
