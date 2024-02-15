package frc.robot.subsystems.leds;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class LEDIOSim implements LEDIO
{
    private AddressableLED       _led;
    private AddressableLEDBuffer _ledBuffer;
    private AddressableLEDSim    _ledSim;
    private ArrayList<Color>     _pattern;

    public LEDIOSim()
    {
        _led       = new AddressableLED(0);
        _ledBuffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);
        _ledSim    = new AddressableLEDSim(_led);

        _led.setLength(_ledBuffer.getLength());
        _led.setData(_ledBuffer);
        _ledSim.setRunning(true);
    }

    @Override
    public void applyAnimationFrame(ArrayList<Color> pattern)
    {
        for (int i = 0; i < pattern.size(); i++)
        {
            var color = pattern.get(i);

            if (color != null)
            {
                _ledBuffer.setLED(i, color);
            }
        }

        _led.setData(_ledBuffer);
        _pattern = pattern;
    }

    @Override
    public ArrayList<Color> getLEDs()
    {
        return _pattern;
    }
}
