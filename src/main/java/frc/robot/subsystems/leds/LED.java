// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.leds;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase
{
    private final LEDIO _io;

    public LED(LEDIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
    }

    public void applyAnimationFrame(Color[] colorList)
    {
        var pattern = new ArrayList<Color>();

        for (int i = 0; i < colorList.length; i++)
        {
            pattern.add(colorList[i]);
        }

        _io.applyAnimationFrame(pattern);
    }

    public ArrayList<Color> getLEDs()
    {
        return _io.getLEDs();
    }
}
