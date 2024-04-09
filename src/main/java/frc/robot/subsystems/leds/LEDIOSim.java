package frc.robot.subsystems.leds;

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
    private Color[]              _pattern;

    public LEDIOSim()
    {
        _led       = new AddressableLED(0);
        _ledBuffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);
        _ledSim    = new AddressableLEDSim(_led);

        _led.setLength(_ledBuffer.getLength());
        _led.setData(_ledBuffer);
        _led.start();
        _ledSim.setRunning(true);
    }

    @Override
    public void updateInputs(LEDIOInputs inputs)
    {
        inputs.r   = _pattern != null ? (double)_pattern[0].red : 0;
        inputs.g   = _pattern != null ? (double)_pattern[0].green : 0;
        inputs.b   = _pattern != null ? (double)_pattern[0].blue : 0;
        inputs.hex = _pattern != null ? _pattern[0].toHexString() : "";
    }

    @Override
    public void applyAnimationFrame(Color[] pattern)
    {
        for (int i = 0; i < pattern.length; i++)
        {
            var color = pattern[i];

            if (color != null)
            {
                _ledBuffer.setLED(i, color);
            }
        }

        _led.setData(_ledBuffer);
        _pattern = pattern;
    }

    @Override
    public Color[] getLEDs()
    {
        return _pattern;
    }
}
