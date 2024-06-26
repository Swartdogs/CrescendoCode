// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.leds;

import java.util.Random;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase
{
    private final LEDIO                 _io;
    private final LEDIOInputsAutoLogged _inputs = new LEDIOInputsAutoLogged();
    Random                              rand    = new Random();

    public LED(LEDIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("LED", _inputs);
    }

    public void applyAnimationFrame(Color[] colorList)
    {
        // ArrayList<Color> pattern = new ArrayList<Color>();

        // for (int i = 0; i < colorList.length; i++)
        // {
        // pattern.add(colorList[i]);
        // }

        _io.applyAnimationFrame(colorList);
    }

    public Color[] getLEDs()
    {
        return _io.getLEDs();
    }

    public void switchDefaultCommand(Command defaultCommand)
    {
        var currentDefaultCommand = getDefaultCommand();

        if (currentDefaultCommand != null)
        {
            currentDefaultCommand.cancel();
        }

        setDefaultCommand(defaultCommand);
    }

    public Color[] getRandomColoring(Color color1, Color color2, Color color3)
    {
        var threeColors = new Color[] { color1, color2, color3 };

        var colorList = new Color[Constants.LED.NUM_LEDS];

        for (var i = 0; i < Constants.LED.NUM_LEDS; i++)
        {
            colorList[i] = threeColors[rand.nextInt(3)];
        }

        return colorList;
    }
}
