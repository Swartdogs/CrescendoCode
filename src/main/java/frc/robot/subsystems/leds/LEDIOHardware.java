// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOHardware implements LEDIO
{
    private AddressableLED       _led;
    private AddressableLEDBuffer _ledBuffer;
    private Color[]              _pattern;

    public LEDIOHardware()
    {
        _led = new AddressableLED(0);

        _ledBuffer = new AddressableLEDBuffer(22);

        _led.setLength(_ledBuffer.getLength());
        _led.setData(_ledBuffer);
        _led.start();
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
