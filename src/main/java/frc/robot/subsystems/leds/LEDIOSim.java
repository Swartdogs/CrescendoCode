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
    public void applyAnimationFrame(ArrayList<Color> colorArray)
    {
        for (int i = 0; i < colorArray.size(); i++)
        {
            var color = colorArray.get(i);

            if (color != null)
            {
                _ledBuffer.setLED(i, color);
            }
        }

        _led.setData(_ledBuffer);
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer)
    {
        _ledBuffer = buffer;

        _led.setData(buffer);
    }

    @Override
    public AddressableLEDBuffer getLEDs()
    {
        return _ledBuffer;
    }
}
