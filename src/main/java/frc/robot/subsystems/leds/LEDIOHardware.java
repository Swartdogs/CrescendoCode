// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.leds;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class LEDIOHardware implements LEDIO
{
    private AddressableLED       _led;
    private AddressableLEDBuffer _ledBuffer;
    private ArrayList<Color>     _pattern;

    public LEDIOHardware()
    {
        _led       = new AddressableLED(0);

        _ledBuffer = new AddressableLEDBuffer(22);

        _led.setLength(_ledBuffer.getLength());
        _led.setData(_ledBuffer);
        _led.start();
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
